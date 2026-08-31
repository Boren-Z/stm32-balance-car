#include "stm32f10x.h"
#include "PID.h"

/**
 * ================================================================
 * 【五层塔定位】算法层 - 通用PID控制器模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   调度层    → 不直接调用PID，通过Control层间接驱动
 *   算法层    ← 本模块，提供可复用的离散PID计算单元
 *   反馈层    → Control层从反馈层读取FB值后传入PID_Update()
 *
 * 【本模块的核心设计哲学】
 *   PID是纯粹的计算工具，不知道自己被用在哪个环
 *   不知道电机的物理限制，不知道车身的物理参数
 *   只做一件事：给定SP和FB，算出Output
 *   这是"单一职责原则"在控制算法上的体现
 *
 * 【为什么需要多实例，而不是单例static模块】
 *   平衡车需要三个PID环（速度/角度/角速度）
 *   同一套算法逻辑，三套独立的参数和状态
 *   必须通过结构体指针传参区分"操作哪一个实例"
 *   类比CODESYS：同一个FB类型，实例化三次
 *     pid_velocity : PID_FB;
 *     pid_theta    : PID_FB;
 *     pid_theta_dot: PID_FB;
 *   这正是结构体指针传参（多实例）
 *   vs static内部变量（单例）的根本区别
 *
 * 对外接口：
 *   PID_Init()      → Action接口，初始化参数
 *   PID_SetSP()     → Set接口，外环写入目标值
 *   PID_Update()    → Update接口，推动一步计算
 *   PID_GetOutput() → Get接口，读取最新输出
 *   PID_Reset()     → Action接口，复位内部状态
 *
 * 【两个结构体的分工】
 *   PID_Para_t  → VAR_CONSTANT：Kp/Ki/Kd/DT/积分限幅
 *                 初始化后不变，换不同参数就是换不同的调参方案
 *   PID_State_t → VAR：SP/误差系列/积分/Output
 *                 每个周期都在变化的运行状态
 *   分开的理由：复位时只清State，不碰Para——
 *               调好的参数不应该因为系统复位而丢失
 * ================================================================
 */

/**
 * ================================================================
 * 【纵横术——个人心得】
 * ================================================================
 *
 * 合纵（纵向）= 五层塔架构：
 *   PID模块在算法层，向上只暴露接口，向下只接收反馈值
 *   不知道驱动层怎么读传感器，不知道调度层怎么安排时序
 *   纵向数据流：反馈层Get → Control层 → PID_Update(FB) → PID_GetOutput()
 *                                                              → Motor_Speed_Set()
 *
 * 连横（横向）= 功能块内部分工：
 *   Init  → 只调一次，写入Para（VAR_CONSTANT）
 *   SetSP → 接收外环输出，写入State.SP（VAR_INPUT）
 *   Update → 推动计算，更新State（VAR）
 *   Get   → 导出Output（VAR_OUTPUT）
 *   Reset → 清零State，不碰Para
 *
 * 关于SP的归属——从VAR_INPUT到VAR的思考：
 *   SP是外部写入的，语义上是VAR_INPUT
 *   但它需要跨调用保留（SetSP和Update不在同一时刻调用）
 *   所以物理上存在State结构体里（VAR），通过SetSP接口写入
 *   这正是"周期边界更新"原则：
 *   SetSP只是"写进缓冲"，Update执行时才真正用到
 *   和TIM影子寄存器、CODESYS扫描周期输出更新是同一个底层规律
 * ================================================================
 */

/**
 * ================================================================
 * [算法层] PID初始化（对外Action接口）
 * ================================================================
 *
 * 【曾有的疑问】
 *   为什么PID_Init()传散装float参数，而不是直接传一个已填好的PID_Para_t结构体？
 *   → Init的职责是"从外部把参数写进结构体"
 *   → 如果传结构体，调用方就要自己先手动填结构体，Init就没意义了
 *   → 散装参数传入，Init负责打包存储，调用方只需要知道数值
 *   类比CODESYS：配置FB参数时逐字段填写，不是把整个VAR块传进去
 *
 * 【关于DT为什么放在PID_Para_t里，而不是头文件的宏定义】
 *   最初想把dt定义为#define COMPLEMENTARY_DT 0.005f
 *   但三个环的调用频率不同（速度环50ms，角度/角速度环5ms）
 *   一个固定宏无法服务不同频率的环
 *   把DT放进Para_t，初始化时各自传入自己的调用周期
 *   同一套PID代码才能真正被不同频率的环复用
 * ================================================================
 */
void PID_Init(PID_Para_t* para, float kp, float ki, float kd,
              float upper, float lower, float dt)
{
    para->KP = kp;
    para->KI = ki;
    para->KD = kd;
    para->Integral_UpperLimit = upper;
    para->Integral_LowerLimit = lower;
    para->DT = dt;
}

/**
 * ================================================================
 * [算法层] 设置目标值（对外Set接口）
 * ================================================================
 *
 * 职责：外环调用，把自己的输出写入内环的SP
 * 这是串级PID中"环与环之间传递数据"的标准方式
 *
 * 【曾有的疑问】
 *   SP应该放在PID_Para_t还是PID_State_t？
 *   → SP是外部动态写入的（每50ms速度环会更新角度环的SP）
 *   → 语义上是VAR_INPUT，物理上存在State里
 *   → 通过SetSP接口写入，符合"接口封装内部状态"原则
 *   → 外部不能直接访问state->SP，只能通过SetSP()
 * ================================================================
 */
void PID_SetSP(PID_State_t* state, float sp)
{
    state->SP = sp;
}

/**
 * ================================================================
 * [算法层] PID计算更新（对外Update接口，每个控制周期调用）
 * ================================================================
 *
 * 离散PID公式：
 *   err      = SP - FB                        （当下：比例基础）
 *   err_int += err × dt                       （过去：积分累积）
 *   err_dev  = (err - err_prev) / dt          （未来：微分预判）
 *   Output   = Kp×err + Ki×err_int + Kd×err_dev
 *
 * 【曾有的疑问：为什么积分项要乘dt？MSD里没有乘过】
 *   连续积分：∫e(t)dt，离散化后每步累加 e[k]×Δt
 *   MSD里漏乘dt，等于把dt吸收进Ki了
 *   这样Ki的物理意义变了：Ki_实际 = Ki_理论 × dt
 *   不同调用频率下同一套参数效果会不一样，跨平台复用时出问题
 *   乘了dt之后Ki才是标准的积分增益，参数物理意义正确
 *
 * 【曾有的疑问：微分项为什么要除dt？MSD里只用了err-err_prev】
 *   (err - err_prev)只是差值，没有时间维度
 *   完整的离散微分：变化率 = 差值 / 时间间隔
 *   MSD版本把dt吸收进Kd里了，和积分项的问题一样
 *
 * 【曾有的疑问："当下、过去、未来"和PID三项的对应】
 *   P = 当下：对当前误差的即时响应
 *   I = 过去：历史误差的累积，消除稳态偏差
 *   D = 未来：误差变化趋势的预判，提前"刹车"防超调
 *
 * 【关于输出限幅的设计决策】
 *   最初把输出限幅放在PID_Update()里
 *   后来发现：输出范围是系统约束（电机物理限制），不是PID算法的一部分
 *   PID是纯粹的计算工具，不应该知道执行器的物理限制
 *   最终决定：输出限幅由Control.c在调用PID_GetOutput()后负责
 *   积分限幅保留在Update()里——这是anti-windup，属于算法内部的事
 *
 *   备选方案：输出限幅也放在PID_Para_t里
 *   → 优点：PID自包含
 *   → 缺点：耦合了执行器物理约束，不够纯粹
 *   → 当前选择：由Control.c负责输出限幅
 * ================================================================
 */
void PID_Update(PID_State_t* state, PID_Para_t* para, float FB)
{
    /* 1. 计算当前误差 */
    state->Error = state->SP - FB;

    /* 第一次调用保护：只记录误差，不计算输出 */
    if(state->is_first)
    {
        state->is_first = 0;
        state->Error_Previous = state->Error;  // 记录当前误差
        return;
    }

    float Error_Derivative;

    /* 2. 积分累加（梯形积分，对齐标准代码）*/
    // state->Error_Integral += state->Error * para->DT;  //旧矩形积分
    state->Error_Integral += (state->Error + state->Error_Previous) * para->DT * 0.5f;

    /* 3. 微分计算 */
    Error_Derivative = (state->Error - state->Error_Previous) / para->DT;

    /* 4. 计算输出 */
    state->Output = para->KP * state->Error
                  + para->KI * state->Error_Integral
                  + para->KD * Error_Derivative;

    /* 5. 积分限幅 */
    if(state->Error_Integral > para->Integral_UpperLimit)
        state->Error_Integral = para->Integral_UpperLimit;
    if(state->Error_Integral < para->Integral_LowerLimit)
        state->Error_Integral = para->Integral_LowerLimit;

    /* 6. 更新上次误差 */
    state->Error_Previous = state->Error;
}

/**
 * ================================================================
 * [算法层] 读取PID输出（对外Get接口）
 * ================================================================
 *
 * 职责：只读取已经算好的Output，不触发任何计算
 * 这是VAR_OUTPUT的"导出"操作——工作台上的结果对外只读窗口
 * ================================================================
 */
float PID_GetOutput(PID_State_t* state)
{
    return state->Output;
}

/**
 * ================================================================
 * [算法层] 复位PID状态（对外Reset接口）
 * ================================================================
 *
 * 职责：清零所有运行状态，不影响已配置的参数
 * 类比CODESYS：FB复位只清VAR，不碰VAR_CONSTANT
 *
 * 【关于SP是否归零的设计决策】
 *   SP归零意味着"目标值重置为0"
 *   对平衡车来说：速度目标归零=让车停下来，这是合理的安全行为
 *   复位通常发生在异常或模式切换时，归零SP是安全的默认行为
 * ================================================================
 */
void PID_Reset(PID_State_t* state)
{
    state->SP             = 0;
    state->Error          = 0;
    state->Error_Integral = 0;
    state->Error_Previous = 0;
    state->Output         = 0;
    state->is_first       = 1;
}


