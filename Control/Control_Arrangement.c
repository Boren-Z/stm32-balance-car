#include "stm32f10x.h"
#include "Control_Arrangement.h"
#include "ADC_Battery.h"  // 需要Battery_GetVoltage()
#include "PID.h"
#include "ComplementaryFilter.h"
#include "EncoderFeedback.h"
#include "SYSTEM_TIM.h"
#include "Motor_PWM.h"  // 加在文件顶部
#include <math.h>
#include <stdio.h>
#include "debugging_USART.h"
#include "Motor_Encoder.h"


extern float omega_ref;

/**
 * ================================================================
 * 【五层塔定位】算法层 - 串级PID控制逻辑模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   调度层    → 调用Control_Update()（每5ms）和Control_SetMoveSpeed()
 *   算法层    ← 本模块，串联三个PID环，最终驱动电机
 *   反馈层    → 本模块调用ComplementaryFilter和EncoderFeedback的Get接口
 *   驱动层    → 本模块调用Motor_Speed_Set()输出控制量
 *
 * 【本模块的角色：统筹协调者】
 *   不做具体的数学计算（那是PID.c的事）
 *   不做传感器读取（那是反馈层的事）
 *   只负责：把正确的数据，在正确的时机，传给正确的环
 *   类比CODESYS主程序：调用各个FB实例，串联它们的输入输出
 *
 * 【三环串级结构与物理意义】
 *   速度环（最外，50ms）：
 *     FB = 实际线速度（编码器）
 *     SP = 目标速度（蓝牙设置，静止时=0）
 *     Output = 目标倾角(theta_ref)
 *     物理意义：车要减速就需要后仰，要加速就需要前倾
 *
 *   角度环（中环，5ms）：
 *     FB = 实际倾角（互补滤波）
 *     SP = 速度环输出的目标倾角
 *     Output = 目标角速度(theta_dot_ref)
 *
 *   角速度环（最内，5ms）：
 *     FB = 实际角速度（陀螺仪GyroX）
 *     SP = 角度环输出的目标角速度
 *     Output = 最终PWM控制量
 *
 * 【为什么三环不会"打架"】
 *   串级PID的关键：外环输出是内环的SP，不是直接控制电机
 *   最终控制电机的只有最内环的输出，不存在两个环争夺PWM的情况
 *   内环必须比外环快（响应倒下比响应漂移需要更快）
 *   这是物理时间尺度决定的，不是人为规定的
 *
 * 对外接口：
 *   Control_Init()         → Action接口，初始化三个PID环参数
 *   Control_Update()       → Update接口，调度层每5ms调用
 *   Control_SetMoveSpeed() → Set接口，蓝牙/外部设置目标速度
 *   Control_Reset()        → Action接口，复位所有PID状态
 *
 * 【没有Get接口的原因】
 *   Control_Update()的输出直接通过Motor_Speed_Set()作用于硬件
 *   没有下游模块来读取这个结果，不需要对外暴露Get接口
 *   类比：ComplementaryFilter需要GetPitch()因为算法层要读它
 *         Control不需要Get因为它的输出直接就是电机动作
 * ================================================================
 */

/**
 * ================================================================
 * 【纵横术——个人心得】
 * ================================================================
 *
 * 从底层驱动层开始，一块寄存器一块寄存器地啃
 * 那时候只关心"这个外设怎么配置"，每个模块独立存在
 * 没有层的概念，没有数据流的意识
 *
 * 到了算法层，第一次真正面对"我需要从下层拿数据、向下层发指令"
 * 这时候才真正理解了"合纵连横"——
 *
 * 合纵（纵向）= 五层塔：
 *   层与层之间只通过接口通信，不跨层
 *   驱动层不知道算法层的存在，算法层不知道寄存器的存在
 *   这正是"关注点分离"在控制系统上的具体化
 *
 * 连横（横向）= 功能块分工：
 *   Init / Set / Update / Get / Reset
 *   VAR_INPUT / VAR / VAR_OUTPUT
 *   单例（ComplementaryFilter）vs 多实例（PID）
 *
 * 纵横交叉点 = 接口函数：
 *   Get  → 纵向输出口
 *   Set  → 纵向输入口
 *   Update → 横向执行引擎
 *
 * 这套理论不只适用于平衡车，不只适用于C语言
 * CODESYS/TwinCAT/ROS都是同一套思想的不同实现
 * 本质是工程复杂度管理的普世方法论
 *
 * 理论不是先学后用，而是从实战中归纳出来的
 * 从驱动层到反馈层到算法层，羽翼渐丰，方能悟道
 * ================================================================
 */

/* ============================================================
 * VAR_INPUT（通过反馈层Get接口读取）：
 *   Speed_FB.Left_Speed  → 左轮速度，来自EncoderFeedback
 *   Speed_FB.Right_Speed → 右轮速度，来自EncoderFeedback
 *   Imu_FB.Pitch         → 实际倾角，来自ComplementaryFilter
 *   Imu_FB.PitchDot      → 实际角速度，来自ComplementaryFilter
 *
 * VAR（内部状态，static）：
 *   Para_Velocity/State_Velocity → 速度环PID实例
 *   Para_Theta/State_Theta       → 角度环PID实例
 *   Para_ThetaDot/State_ThetaDot → 角速度环PID实例
 *   Last_Velocity                → 速度环50ms计时器
 *
 * VAR_OUTPUT：
 *   无（输出直接通过Motor_Speed_Set()写入驱动层，不对外暴露）
 * ============================================================ */

static PID_Para_t  Para_Velocity;
static PID_State_t State_Velocity;

static PID_Para_t  Para_Theta;
static PID_State_t State_Theta;

static PID_Para_t  Para_ThetaDot;
static PID_State_t State_ThetaDot;

static PID_Para_t  Para_MotorOmega_L;
static PID_State_t State_MotorOmega_L;

static PID_Para_t  Para_MotorOmega_R;
static PID_State_t State_MotorOmega_R;

static PID_Para_t  Para_Turn;   // 转向环参数
static PID_State_t State_Turn;  // 转向环状态
static float omega_ref = 0.0f;  // 加在函数开头


float Control_GetOmegaRef(void)
{
    return omega_ref;
}


/**
 * ================================================================
 * [算法层] 控制初始化（对外Action接口）
 * ================================================================
 *
 * 【参数来源】
 *   直接使用铁头山羊标准代码的参数，未经自行推导
 *   速度环DT=0.05f（50ms），角度/角速度环DT=0.005f（5ms）
 *   DT放在Para_t里而不是全局宏——不同频率的环需要各自的dt
 *
 * 【积分限幅的选择】
 *   速度环：±0.5g（约±4.9rad），对应合理的目标倾角范围
 *   角度环：±4π rad/s（约±12.57），对应合理的目标角速度范围
 *   角速度环：±40π rad/s²（约±125.7），对应合理的角加速度范围
 * ================================================================
 */
void Control_Init(void)
{    // 速度环（最外环，5ms）
    PID_Init(&Para_Velocity, 7.0f, 2.0f, 0.0f, 0.5f*9.8f, -0.5f*9.8f, 0.005f);

    // 角度环（中环，5ms）
    PID_Init(&Para_Theta,     8.0f,  0.0f,  0.0f, 12.57f,    -12.57f,    0.005f);

    // 角速度环（最内环，5ms）
    PID_Init(&Para_ThetaDot,  10.0f, 10.0f, 0.0f, 125.7f,    -125.7f,    0.005f);

    // 电机速度环（第四环，1ms）Kp=0.5, Ki=7，输出限幅±8.4V（对应满电电池电压）输出单位是电压V，不是PWM占空比
    PID_Init(&Para_MotorOmega_L, 0.5f, 3.0f, 0.0f, 8.4f, -8.4f, 0.001f);
    PID_Init(&Para_MotorOmega_R, 0.5f, 3.0f, 0.0f, 8.4f, -8.4f, 0.001f);

    PID_Init(&Para_Turn,      1.0f,  0.0f,  0.0f, 15.0f, -15.0f, 0.005f);  

}
/**
 * ================================================================
 * [算法层] 控制更新（对外Update接口，调度层每5ms调用）
 * ================================================================
 *
 * 串级执行顺序：
 *   1. 从反馈层读取所有反馈量
 *   2. 速度环（50ms）：计算目标倾角 → SetSP给角度环
 *   3. 角度环（5ms）：计算目标角速度 → SetSP给角速度环
 *   4. 角速度环（5ms）：计算最终控制量 → Motor_Speed_Set()
 *
 * 【速度环为什么50ms，角度环为什么5ms】
 *   倒下的速度比漂移的速度快10倍以上
 *   内环（快变量）必须比外环（慢变量）响应快
 *   这是物理时间尺度决定的，不是人为规定的
 *
 * 【速度环50ms的实现方式】
 *   Control_Update()本身每5ms被调用一次
 *   速度环内部有独立的50ms计时器
 *   10次调用里只有1次真正执行速度环
 *   角度环和角速度环每次都执行
 *
 * 【曾犯的错误】
 *   1. 角度环和角速度环也加了5ms计时器
 *      Control_Update()本身就是5ms调用，内部再加5ms if等于永远成立
 *      多余的计时框架直接删掉
 *
 *   2. Motor_Speed_Set()语法错误
 *      float PWM_ref = Motor_Speed_Set(int16_t PWM_ref, int16_t PWM_ref)
 *      Motor_Speed_Set是void类型，不返回值，参数不能带类型声明
 *      正确：float output = PID_GetOutput(&State_ThetaDot);
 *            Motor_Speed_Set((int16_t)output, (int16_t)output);
 *
 *   3. 角速度环最后多写了一行PID_SetSP()
 *      角速度环是最内环，输出直接给电机，不需要再SetSP给任何东西
 * ================================================================
 */
void Control_Update(void)
{
    // Motor_Speed_Set(0, 0);  // 临时禁用，专注测试编码器
    // return;
    // ================================================================
    // 按键使能（PA11，低电平有效）
    // 开机默认电机关闭，按一次按键才启动控制
    // 和标准代码app_button.c的行为一致：单击翻转电机状态
    // ================================================================
    static uint8_t motor_enabled = 0;
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET)
    {
        motor_enabled = !motor_enabled;
        Control_Reset();
        // 等待按键松开，简单去抖
        while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET);
    }

    if(motor_enabled == 0)
    {
        Motor_Speed_Set(0, 0);
        return;
    }

    // ================================================================
    // 第一步：从反馈层读取所有反馈量
    // 严格遵守五层塔原则：算法层只调反馈层接口，不跨层访问驱动层
    // ================================================================
    SpeedFeedback  Speed_FB;
    IMU_Feedback_t Imu_FB;
    EncoderFeedback_GetSpeed(&Speed_FB);
    ComplementaryFilter_GetFeedBack(&Imu_FB);

    // ================================================================
    // 倾角安全保护
    // 倾角超过45°立刻停止，防止摔倒时电机全速运转
    // 标准代码没有这一步，这是我们额外加的安全措施
    // ================================================================
    /* 旧方案（开环估算，注释保留备用）
    int16_t pwm = (int16_t)(omega_ref * 10.0f);
    if(pwm >  100) pwm =  100;
    if(pwm < -100) pwm = -100;
    Motor_Speed_Set(pwm, pwm);
    */

    // 新方案：把omega_ref作为目标轮速传给第四环
    // omega_ref单位rad/s，第四环PID负责把它转换成实际PWM
    // PID_SetSP(&State_MotorOmega_L, omega_ref);
    // PID_SetSP(&State_MotorOmega_R, omega_ref);


    // ================================================================
    // 第二步：单位换算——度→弧度
    //
    // 【曾犯的严重错误】
    //   最初直接把Imu_FB.Pitch（单位°）传给角度环PID
    //   但标准代码的PID参数（Kp=4）是按弧度调的
    //   1rad ≈ 57.3°，同样的误差用度数算出来的值是弧度的57倍
    //   导致PID输出严重过大，车会剧烈震荡无法站立
    //   必须先换算成弧度才能和参数的物理意义匹配
    //
    // PitchDot已经在ComplementaryFilter里换算成rad/s了，不需要再换
    // ================================================================
    float theta     = Imu_FB.Pitch * (3.1415926f / 180.0f);
    float theta_dot = Imu_FB.PitchDot;

    // ================================================================
    // 第三步：速度修正——剔除倾斜产生的"虚假轮速"
    //
    // 【为什么需要这一步】
    //   平衡车车身倾斜时，即使没有真正向前平移，
    //   重心的运动也会带动轮子转动——这部分转动是"被迫的"，
    //   不代表车真的在往前跑。
    //   如果不剔除，速度环会误判实际速度，做出错误的补偿。
    //
    // 【物理推导】
    //   重心绕轮轴转动产生的虚假轮速：
    //   omega2 = -theta_dot × (lp+rw) / rw
    //     lp = 轮心到重心的距离（0.062m）
    //     rw = 轮子半径（0.032m）
    //     负号：倾斜方向和轮速方向相反
    //
    //   真实轮速 = 测量轮速 - 虚假轮速
    //   真实线速度 = 真实轮速 × 轮子半径
    // ================================================================
    static const float g  = 9.8f;
    static const float lp = 0.062f;
    static const float rw = 0.032f;

    float omega  = 0.5f * (Speed_FB.Left_Speed + Speed_FB.Right_Speed);
    float omega2 = -theta_dot * (lp + rw) / rw;
    float omega1 = omega - omega2;
    float x_dot  = omega1 * rw;

    // ================================================================
    // 第四步：速度环（50ms执行一次）
    //
    // 【为什么速度环比角度环慢10倍】
    //   车身倒下的时间尺度：几百毫秒内就会摔倒，必须5ms快速响应
    //   车身漂移的时间尺度：几秒钟才会跑偏，50ms足够
    //   内环必须比外环快——这是物理时间尺度决定的，不是人为规定的
    //
    // 【速度环输出→目标倾角的换算：为什么用atan(a/g)】
    //   速度环PID输出的是"需要多大的线加速度a（m/s²）"
    //   但角度环需要的是"目标倾角theta_ref（rad）"
    //   两者的物理关系：平衡车要产生加速度a，需要倾斜theta_ref
    //   由牛顿第二定律：F = ma，水平力 = mg×tan(theta)
    //   因此：a = g×tan(theta) → theta = atan(a/g)
    // ================================================================
    static uint32_t Last_Velocity = 0;
    uint32_t now = System_GetTick();
    // if(now - Last_Velocity >= 50)
    if(now - Last_Velocity >= 5)
    {
        Last_Velocity = now;
        PID_Update(&State_Velocity, &Para_Velocity, x_dot);
        float theta_ref = atanf(PID_GetOutput(&State_Velocity) / g);
        PID_SetSP(&State_Theta, theta_ref);
    }

    // ================================================================
    // 第五步：角度环（5ms）
    //
    // FB = theta（实际倾角，弧度）
    // SP = theta_ref（速度环输出的目标倾角，弧度）
    // Output = theta_dot_ref（目标角速度，rad/s）
    // ================================================================
    PID_Update(&State_Theta, &Para_Theta, theta);
    float theta_dot_ref = PID_GetOutput(&State_Theta);
    PID_SetSP(&State_ThetaDot, theta_dot_ref);

    // ================================================================
    // 第六步：角速度环（5ms）
    //
    // FB = theta_dot（实际角速度，rad/s）
    // SP = theta_dot_ref（角度环输出的目标角速度，rad/s）
    // Output = theta_dot_dot_ref（目标角加速度，rad/s²）
    // ================================================================
    PID_Update(&State_ThetaDot, &Para_ThetaDot, theta_dot);
    float theta_dot_dot_ref = PID_GetOutput(&State_ThetaDot);

    // ================================================================
    // 第七步：逆解算→轮速→PWM
    //
    // 倒立摆运动方程：
    //   x_dot_dot = (g×sin(theta) - theta_dot_dot×lp) / cos(theta)
    // 线加速度积分得轮速：
    //   omega_ref += (1/rw) × x_dot_dot × dt
    //
    // 【omega_ref限幅】
    //   防止积分器无限累积导致电机饱和
    //   ±10 rad/s对应PWM ±100，是物理合理范围
    // ================================================================
    float x_dot_dot_ref = (g * sinf(theta) - theta_dot_dot_ref * lp) / cosf(theta);

    static uint32_t last_time = 0;
    float dt = (now - last_time) * 0.001f;
    if(last_time != 0)
    {
        omega_ref += (1.0f / rw) * x_dot_dot_ref * dt;
    }
    last_time = now;

        // omega_ref限幅——防止积分器失控
    if(omega_ref >  7.0f) omega_ref =  7.0f;
    if(omega_ref < -7.0f) omega_ref = -7.0f;

    /* 旧方案（开环，注释保留备用）
    int16_t pwm = (int16_t)(omega_ref * 10.0f);
    if(pwm >  100) pwm =  100;
    if(pwm < -100) pwm = -100;
    Motor_Speed_Set(pwm, pwm);
    */

    // 转向环：读取偏航角速度，计算差速修正量
    PID_Update(&State_Turn, &Para_Turn, Imu_FB.YawDot);
    float omega_diff = PID_GetOutput(&State_Turn);

    // 左右轮差速输出（转向环修正）
    PID_SetSP(&State_MotorOmega_L, omega_ref + omega_diff);
    PID_SetSP(&State_MotorOmega_R, omega_ref - omega_diff);
}

void Control_Motor_Update(void)
{
    // ================================================================
    // 第四环：电机速度闭环（1ms执行）
    //
    // 【为什么需要第四环】
    //   三环控制逻辑输出的是omega_ref（目标轮速，rad/s）
    //   但Motor_Speed_Set()需要的是PWM占空比（-100~100）
    //   直接用omega_ref×系数是开环的，电池电压变化会导致：
    //     同样的PWM，满电时转速快，快没电时转速慢
    //     控制效果随电池电压漂移，无法稳定平衡
    //
    // 【第四环的工作原理】
    //   SP = omega_ref（三环输出的目标轮速）
    //   FB = 实际轮速（编码器测量）
    //   PID输出 = 电压Ua（单位V）
    //   PWM占空比 = Ua / Vbat × 100%
    //   → 除以电池电压，补偿电压变化，保证转速稳定
    // ================================================================


    // int16_t pwm_l = (int16_t)(State_MotorOmega_L.SP * 10.0f);
    // int16_t pwm_r = (int16_t)(State_MotorOmega_R.SP * 10.0f);
    // if(pwm_l >  100) pwm_l =  100;
    // if(pwm_l < -100) pwm_l = -100;
    // if(pwm_r >  100) pwm_r =  100;
    // if(pwm_r < -100) pwm_r = -100;
    // Motor_Speed_Set(pwm_l, pwm_r);


    // // 动态deltaT（对齐标准代码）
    // static uint64_t lastTime = 0;
    // uint64_t now = System_GetUs();
    // float deltaT = (now - lastTime) * 1.0e-6f;
    // lastTime = now;
    
    // if(deltaT <= 0 || deltaT > 0.1f) return;  // 第一次调用或异常跳过

    // 读取实际轮速（反馈层接口）
    SpeedFeedback Speed_FB;
    EncoderFeedback_GetSpeed(&Speed_FB);
    // Speed_FB.Left_Speed  = Encoder_Get_L_Speed();
    // Speed_FB.Right_Speed = Encoder_Get_R_Speed();
    // // 更新动态deltaT（对齐标准代码）← 加在这里
    // Para_MotorOmega_L.DT = deltaT;
    // Para_MotorOmega_R.DT = deltaT;

    // 第四环PID计算，输出电压Ua（单位V）
    PID_Update(&State_MotorOmega_L, &Para_MotorOmega_L, Speed_FB.Left_Speed);
    PID_Update(&State_MotorOmega_R, &Para_MotorOmega_R, Speed_FB.Right_Speed);

    float ua_l = PID_GetOutput(&State_MotorOmega_L);
    float ua_r = PID_GetOutput(&State_MotorOmega_R);

    // 读取电池电压，补偿电压变化
    // float vbat = 8.0f;  // 临时固定
    float vbat = Battery_GetVoltage();
    if(vbat < 6.0f) vbat = 6.0f;  // 防止除以极小值

    // char buf[32];
    // sprintf(buf, "vbat:%.2f\r\n", vbat);
    // Debugging_USART_SendString(buf);

    // 临时：固定vbat为ST-Link实际电压
    // float vbat = 3.0f;  // ST-Link供电约3V

    // 换算成PWM占空比输出
    int16_t duty_l = (int16_t)(ua_l / vbat * 100.0f);
    int16_t duty_r = (int16_t)(ua_r / vbat * 100.0f);

    // 临时打印
    // char buf[80];
    // sprintf(buf, "SP_L:%.2f FB_L:%.2f ua_l:%.2f duty_l:%d\r\n",
    //         State_MotorOmega_L.SP, Speed_FB.Left_Speed, ua_l, duty_l);
    // Debugging_USART_SendString(buf);


    Motor_Speed_Set(duty_l, duty_r);
}











/**
 * ================================================================
 * [算法层] 设置目标速度（对外Set接口，蓝牙/外部调用）
 * ================================================================
 *
 * 职责：接收外部（蓝牙遥控）的速度指令，写入速度环SP
 * 静止时SP=0，车会尽量保持原地不动
 * 蓝牙发来"move 0 30"时，这里会被调用，SP=0.3×某个换算系数
 * ================================================================
 */
void Control_SetMoveSpeed(float speed)
{
    PID_SetSP(&State_Velocity, speed);
}

/**
 * ================================================================
 * [算法层] 复位所有PID状态（对外Reset接口）
 * ================================================================
 *
 * 职责：清零三个环的运行状态，不影响已调好的参数
 * 调用时机：模式切换、异常恢复、重新启动平衡时
 * ================================================================
 */
void Control_Reset(void)
{
    omega_ref = 0.0f;
    PID_Reset(&State_Velocity);
    PID_Reset(&State_Theta);
    PID_Reset(&State_ThetaDot);
    PID_Reset(&State_MotorOmega_L);
    PID_Reset(&State_MotorOmega_R);
    PID_Reset(&State_Turn);

}


