#include "stm32f10x.h"
#include "ComplementaryFilter.h"
#include "I2C_MPU6050.h"
#include <math.h>

/**
 * ================================================================
 * 【五层塔定位】反馈处理层 - 互补滤波姿态解算模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   算法层    → 调用ComplementaryFilter_GetPitch()获取俯仰角
 *   反馈层    ← 本模块，把MPU6050原始数据融合成可用的姿态角
 *   驱动层    → 本模块调用I2C_MPU6050的MPU6050_GetData()获取原始数据
 *
 * 物理坐标系（根据实际安装方向确认）：
 *   Y轴 → 车头方向（前进方向）
 *   X轴 → 轮轴方向（左右）
 *   Z轴 → 垂直地面方向（上下）
 *
 *   pitch（俯仰角）= 绕X轴旋转 → 车头上仰/下俯，平衡车需要控制的角度
 *   roll（横滚角） = 绕Y轴旋转 → 车身左右侧倾，安全层监控用（暂不实现）
 *   yaw（偏航角）  = 绕Z轴旋转 → 后期蓝牙转向用，只靠GyroZ积分，无需互补
 *
 * 对外接口（.h文件声明，外部只能看到这三个）：
 *   ComplementaryFilter_Init()     → Action接口，pitch清零
 *   ComplementaryFilter_Update()   → Update接口，Task_Manager每5ms调用
 *   ComplementaryFilter_GetPitch() → Get接口，算法层随时读取最新角度
 *
 * 内部函数（static，外部不可见）：
 *   Filter_ACCL()   → 加速度计路径：重力分量→三角函数→pitch候选值
 *   Filter_Gyro()   → 陀螺仪路径：角速度积分→pitch候选值
 *   Filter_Update() → 互补融合两个候选值，更新Pitch
 *
 * 【互补滤波的核心数学原理】
 *   两条路径"殊途同归"，都能算出pitch，但各有缺陷：
 *
 *   陀螺仪路径（积分法）：
 *     角度 = ∫角速度 dt（一次积分，不是角加速度积分）
 *     优点：短时间内平滑准确
 *     缺点：固有bias被积分累积放大，长期漂移（和PID积分项给大了一样）
 *
 *   加速度计路径（三角函数法）：
 *     角度 = atan2(AccY, AccZ)（不积分，每次独立计算）
 *     优点：长期稳定，不漂移
 *     缺点：瞬时容易被运动/震动干扰，高频抖动大
 *
 *   互补滤波融合公式：
 *     pitch = α×(pitch_prev + gyro×dt) + (1-α)×accel_pitch
 *     α接近1（如0.98）→ 短期信任陀螺仪，长期靠加速度计修正漂移
 *
 *   从频域看（拉普拉斯变换）：
 *     陀螺仪部分 = 高通滤波器 H_gyro(s) = s/(s+1/τ)
 *     加速度计部分 = 低通滤波器 H_accel(s) = (1/τ)/(s+1/τ)
 *     H_gyro(s) + H_accel(s) = 1（互补，全频段覆盖，无盲区）
 *     类比音箱分频器：低音单元+高音单元合起来还原完整声音
 *
 * 【dt的设计决策】
 *   dt固定为0.005f（5ms常量），不动态测量
 *   理由：PERIODIC(5)宏保证Update()严格每5ms调用一次，dt是可信的固定值
 *   对比：标准代码(铁头山羊版)同样把dt硬编码，虽然有GetUs()但互补滤波里根本没用
 *   当前TIM3时基只有毫秒级精度，已经足够互补滤波使用
 *
 * 【为什么只算pitch，不算roll】
 *   pitch → 电机可以主动修正，必须实时控制
 *   roll  → 两轮同轴，电机物理上无法修正，算出来只能给安全层用，优先级低
 *   yaw   → 后期蓝牙转向用，只靠GyroZ积分，加速度计对yaw没有贡献
 *            （重力竖直向下，绕竖轴转动不改变任何轴的重力分量）
 * ================================================================
 */

/* ============================================================
 * VAR_INPUT（通过MPU6050_GetData()内部读取，不对外暴露）：
 *   Raw_Data.GyroX  → 陀螺仪X轴原始LSB值（绕X轴的角速度，对应pitch变化）
 *   Raw_Data.AccY   → 加速度计Y轴原始LSB值（重力在Y方向的分量）
 *   Raw_Data.AccZ   → 加速度计Z轴原始LSB值（重力在Z方向的分量）
 *
 * VAR_OUTPUT（通过GetPitch()对外暴露）：
 *   Pitch           → 当前俯仰角，单位°，正值=前倾，负值=后仰
 *
 * VAR（内部状态，跨调用持久存在）：
 *   Raw_Data        → 原始传感器数据缓存（每次Update()刷新）
 *   pitch_prev      → 陀螺仪积分路径的上一次角度值（Filter_Gyro内部）
 * ============================================================ */

static MPU6050_Info Raw_Data;   // VAR：内部数据缓存，不暴露给上层

static IMU_Feedback_t Output;


/**
 * ================================================================
 * [反馈层-内部] 加速度计路径：重力分量→pitch候选值（static）
 * ================================================================
 *
 * 原理：车身倾斜时，重力在AccY/AccZ上的分量比例发生变化，
 *       用atan2()反算出倾斜角度
 *       这是每次独立计算的瞬时值，不依赖历史数据，不会漂移
 *
 * 【曾犯的错误】
 *   1. accZ误写成Raw_Data.AccY（复制粘贴失误）
 *      float accZ = Raw_Data.AccY * ACCEL_SENSITIVITY → 应该是AccZ
 *
 *   2. 原始LSB值直接传给atan2f()
 *      必须先乘以ACCEL_SENSITIVITY换算成g，再做三角函数计算
 *      否则atan2f()得到的不是真实倾角
 *
 *   3. 用atan2()而不是atan2f()
 *      atan2是double版本，atan2f是float版本
 *      STM32上float运算效率远高于double，统一用atan2f()
 * ================================================================
 */
static float Filter_ACCL(void)
{
    float accY = Raw_Data.AccY * ACCEL_SENSITIVITY;
    float accZ = Raw_Data.AccZ * ACCEL_SENSITIVITY;
    return atan2f(accY, accZ) * (180.0f / 3.1415926f);
}

/**
 * ================================================================
 * [反馈层-内部] 陀螺仪路径：角速度积分→pitch候选值（static）
 * ================================================================
 *
 * 原理：pitch = pitch_prev + GyroX × 换算系数 × dt
 *       每次调用把角速度积分一步，累加到上次的角度值上
 *
 * 【pitch_prev为什么必须是static】
 *   积分需要"记住上一次的值"才能累加，
 *   普通局部变量每次调用都重置为0，积分就失效了
 *   这和Encoder_Get_L_Speed()里的Prev_Encoder_Left_Num是同一种用法
 *
 * 【曾犯的错误】
 *   1. 最初漏写pitch_prev = pitch_gyro这一行
 *      没有更新上次值，下次调用时pitch_prev永远是0，积分完全失效
 *      症状：pitch只会在第一次调用时有数值，之后永远从0开始积分
 * ================================================================
 */
static float Filter_Gyro(void)
{
    float gyroX = Raw_Data.GyroX * GYRO_SENSITIVITY;
    // 从上次滤波后的pitch开始积分，而不是陀螺仪单独积分的值
    float pitch_gyro = Output.Pitch + gyroX * COMPLEMENTARY_DT;
    return pitch_gyro;
}

/**
 * ================================================================
 * [反馈层-内部] 互补滤波融合（static）
 * ================================================================
 *
 * 公式：pitch = α×陀螺仪候选值 + (1-α)×加速度计候选值
 *   α=0.98：短期信任陀螺仪（高频响应快），长期靠加速度计修正漂移（低频稳定）
 *   α不能=1：没有加速度计修正，陀螺仪漂移永远无法被拉回来
 *   α不能=0：完全依赖加速度计，高频抖动全部进入控制环
 * ================================================================
 */
static float Filter_Update(void)
{
    return ARLFA * Filter_Gyro() + (1 - ARLFA) * Filter_ACCL();
}

/**
 * ================================================================
 * [反馈层] 互补滤波初始化（对外Action接口）
 * ================================================================
 */
void ComplementaryFilter_Init(void)
{
    Output.Pitch = 0;
    Output.PitchDot = 0;
    Output.YawDot = 0;
}

/**
 * ================================================================
 * [反馈层] 互补滤波更新（对外Update接口，Task_Manager每5ms调用）
 * ================================================================
 *
 * 职责：先读取最新原始数据，再触发计算，更新Pitch
 *
 * 【Update接口的设计原则】
 *   Update = "现在去做一次计算，推动状态向前走一步"
 *   由调度层周期性调用，有时序要求
 *
 *   对比Get接口：
 *   Get    = "告诉我现在的值是什么"，只读结果，不触发计算
 *   算法层随时可调用Get，不受时序约束
 *
 * 【曾犯的错误】
 *   最初忘记在Update()里调用MPU6050_GetData()
 *   Raw_Data里永远是初始值(全0)，Filter_ACCL()和Filter_Gyro()
 *   算出来的结果完全没有意义
 *
 * 【为什么反馈层自己主动调驱动层，而不是等数据从外部传入】
 *   曾经的设计：Filter_Complementary(MPU6050_Info* Data)，
 *   调用方需要自己持有数据结构再传进来
 *   改进后的设计：Update()内部主动调MPU6050_GetData()
 *   算法层完全不需要知道MPU6050_Info是什么，接口更干净
 *   这正是五层塔"数据在层内部流动，层间只交换结果"的体现
 * ================================================================
 */
void ComplementaryFilter_Update(void)
{
    MPU6050_GetData(&Raw_Data);
    Output.Pitch    = Filter_Update();
    Output.PitchDot = Raw_Data.GyroX * GYRO_SENSITIVITY * (3.1415926f / 180.0f);
    Output.YawDot   = Raw_Data.GyroZ * GYRO_SENSITIVITY * (3.1415926f / 180.0f); 
}

/**
 * ================================================================
 * [反馈层] 获取当前俯仰角（对外Get接口）
 * ================================================================
 *
 * 职责：只读取已经算好的Pitch值，不触发任何计算
 *
 * 【Get接口的两种形式，本模块选用return形式】
 *   return形式（单个值）：float ComplementaryFilter_GetPitch(void)
 *   指针形式（结构体）：void EncoderFeedback_GetSpeed(SpeedFeedback* Output)
 *   本质相同：都是把内部状态"露出来"给外部的只读窗口
 *   类比CODESYS：读取FB的VAR_OUTPUT，不触发FB执行
 * ================================================================
 */
void ComplementaryFilter_GetFeedBack(IMU_Feedback_t* output)
{
    *output = Output;
}

