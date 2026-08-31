#include "EncoderFeedback.h"

/**
 * ================================================================
 * 【五层塔定位】反馈处理层 - 编码器速度反馈模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   算法层    → 调用EncoderFeedback_GetSpeed()获取轮速
 *   反馈层    ← 本模块，把编码器脉冲增量换算成物理单位的轮速
 *   驱动层    → 本模块调用Motor_Encoder的Encoder_Get_L/R_Speed()
 *
 * 对外接口：
 *   EncoderFeedback_Update()    → Update接口，Task_Manager每5ms调用
 *   EncoderFeedback_GetSpeed()  → Get接口，算法层读取轮速
 *
 * 内部函数（static，外部不可见）：无，逻辑简单直接写在Update()里
 *
 * 【为什么需要反馈层处理，不让算法层直接用驱动层数据】
 *   驱动层Encoder_Get_L_Speed()返回的是"每5ms内的脉冲增量"（纯数字，无物理单位）
 *   算法层PID需要的是"轮速(rad/s)"或"线速度(m/s)"（有物理意义）
 *   反馈层负责这个"原始数字→物理量"的翻译工作，
 *   和ComplementaryFilter把"原始LSB→角度"是同一个分层原则
 *
 * 【换算系数来源——从标准代码里提取的硬件参数】
 *   标准代码第28行：encoder / 22.0f / (30613.0f/1500.0f) * 360.0f
 *   反推出：
 *     编码器线数 = 22（A相单边沿，每转22个脉冲）
 *     减速比     = 30613/1500 ≈ 20.4
 *     轮子每转总脉冲数 = 22 × 20.4 ≈ 449
 *
 * 【M法 vs T法，为什么现在用M法】
 *   M法（本模块）：固定时间窗口(5ms)内数脉冲数，用增量/时间算速度
 *   T法（标准代码）：记录相邻脉冲的微秒级时间戳，用1/T算速度
 *   T法低速精度更好，但需要GetUs()微秒级时基（当前TIM3只有毫秒级）
 *   本着"先实现再改进"的原则，现在用M法先跑起来，
 *   后期引入SysTick微秒级时基后，再升级为T法
 *
 * 【原子保护问题，已知遗留】
 *   标准代码里用了__disable_irq()/__enable_irq()保护编码器变量的读取
 *   本项目的Encoder_Get_L/R_Speed()目前没有原子保护
 *   低速运行时概率较低，但高速时可能读到被中断打断的"半更新"数据
 *   后期和T法升级一起处理
 * ================================================================
 */

/* ============================================================
 * VAR_INPUT（通过Encoder_Get_L/R_Speed()内部读取）：
 *   脉冲增量（int32_t）→ 每5ms内编码器计数变化量
 *
 * VAR_OUTPUT（通过GetSpeed()对外暴露）：
 *   Speed_Output.Left_Speed  → 左轮轮速，单位rad/s
 *   Speed_Output.Right_Speed → 右轮轮速，单位rad/s
 *
 * VAR（内部状态）：
 *   Speed_Output → 最新换算结果缓存
 * ============================================================ */

static SpeedFeedback Speed_Output;   // VAR：内部缓存，通过GetSpeed()对外暴露

/**
 * ================================================================
 * [反馈层] 编码器速度更新（对外Update接口，Task_Manager每5ms调用）
 * ================================================================
 *
 * 换算公式：
 *   轮速(rad/s) = 脉冲增量 / (线数 × 减速比) × 2π / dt
 *   物理意义：脉冲增量÷总脉冲数=转了几圈，×2π=弧度，÷dt=rad/s
 *
 * 【曾犯的错误】
 *   1. 结构体定义写在使用之后
 *      C语言从上往下读，typedef struct必须在static SpeedFeedback之前
 *
 *   2. SpeedFeedback成员类型用了int32_t而不是float
 *      轮速是带小数的浮点数（如1.57 rad/s），int32_t会截断小数部分
 *
 *   3. REDUCTION_RATIO宏定义缺少括号
 *      #define REDUCTION_RATIO 30613.0f/1500.0f（没括号）
 *      展开后运算顺序可能出错，必须加括号：(30613.0f/1500.0f)
 *
 *   4. Encoder_Get_L_Speed()返回int32_t，参与浮点运算需要(float)强制转换
 *      不加转换编译器会做隐式提升，加了更安全、意图更清晰
 *
 *   5. 换算公式里2π位置错误
 *      曾写成：/ (线数 × 减速比 × 2π) / dt（2π在分母）
 *      正确：/ (线数 × 减速比) × (2π / dt)（2π在分子）
 *
 *   6. static SpeedFeedback Speed_Output误放进了头文件
 *      每个include该头文件的.c都会各自生成一份Speed_Output
 *      内部私有变量必须只放在.c文件里
 * ================================================================
 */
void EncoderFeedback_Update(void)
{
    Speed_Output.Left_Speed  = (float)Encoder_Get_L_Speed();
    Speed_Output.Right_Speed = (float)Encoder_Get_R_Speed();
}

/**
 * ================================================================
 * [反馈层] 获取轮速（对外Get接口）
 * ================================================================
 *
 * 职责：把内部Speed_Output结构体拷贝给调用方，不触发任何计算
 *
 * 【指针形式的Get接口，适合多值打包返回】
 *   return形式适合单个值（ComplementaryFilter_GetPitch用的这种）
 *   指针形式适合结构体（多个值打包）
 *   本质相同：都是内部状态对外的只读窗口
 *
 * 【曾犯的错误】
 *   最初在GetSpeed()里调用了EncoderFeedback_Update()
 *   Get函数不应该触发计算——Update由Task_Manager周期调用，
 *   Get只是读取已经算好的结果，不能在Get里再触发一次Update
 * ================================================================
 */
void EncoderFeedback_GetSpeed(SpeedFeedback* Outputs)
{
    *Outputs = Speed_Output;
}

/**
 * =======================旧设计====================================
 * [反馈层] 编码器速度更新（对外Update接口，Task_Manager每5ms调用）
 * ================================================================
 *
 * 换算公式：
 *   轮速(rad/s) = 脉冲增量 / (线数 × 减速比) × 2π / dt
 *   物理意义：脉冲增量÷总脉冲数=转了几圈，×2π=弧度，÷dt=rad/s
 **/

// void EncoderFeedback_Update(void)
// {
//     Speed_Output.Left_Speed  = (float)Encoder_Get_L_Speed()
//                                / (ENCODER_LINE_COUNTE * REDUCTION_RATIO)
//                                * (2 * 3.1415926f / FEEDBACK_DT);

//     Speed_Output.Right_Speed = (float)Encoder_Get_R_Speed()
//                                / (ENCODER_LINE_COUNTE * REDUCTION_RATIO)
//                                * (2 * 3.1415926f / FEEDBACK_DT);
// }
