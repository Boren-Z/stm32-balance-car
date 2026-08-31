#include "stm32f10x.h"
#include "SYSTEM_TIM.h"
/**
 * ================================================================
 * 【五层塔定位】驱动层 - 编码器模块（T法测速）
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   算法层    → 调用Encoder_Get_L/R_Speed()获取实时转速反馈，
 *                未来接入闭环PID控制；Position可用于里程累计
 *   驱动层    ← 本模块，EXTI外部中断实现正交编码器测速测向
 *
 * 硬件架构：
 *   左编码器：ENC_L_A → PB14（EXTI中断触发相）
 *            ENC_L_B → PB15（被动读取相）
 *   右编码器：ENC_R_A → PB3 （EXTI中断触发相）
 *            ENC_R_B → PB4 （被动读取相）
 *
 *   PB3/PB4默认是JTAG调试引脚（TDO/NJTRST），必须先解除JTAG占用
 *   才能用作普通GPIO，同时保留SWD(PA13/14)给ST-Link调试不受影响
 *
 * 对外接口（.h文件声明，外部只能看到这五个）：
 *   Encoder_Init()            → Action接口，初始化GPIO/EXTI/NVIC
 *   Encoder_Get_L_Speed()     → Get接口，左轮M法测速（两次调用间的增量）
 *   Encoder_Get_R_Speed()     → Get接口，右轮M法测速
 *   Encoder_Get_L_Position()  → Get接口，左轮累计脉冲数
 *   Encoder_Get_R_Position()  → Get接口，右轮累计脉冲数
 *
 * 内部函数（static，外部不可见）：
 *   Encoder_GPIO_Init()  → 引脚模式配置+JTAG解除占用
 *   Encoder_EXTI_Init()  → A相双边沿中断配置
 *   Encoder_NVIC_Init()  → 中断优先级配置
 *
 * 【为什么放弃TIM硬件编码器模式，改用EXTI软件M法】
 *   STM32标准的TIM编码器模式需要A/B两相接在同一个定时器的CH1/CH2上
 *   查芯片复用功能表后发现两个编码器都不满足这个条件：
 *     PB14/PB15 → TIM1_CH2N/CH3N（互补通道，不是常规CH1/CH2）
 *     PB3/PB4   → TIM2_CH2/TIM3_CH1（分属两个不同定时器）
 *   两组引脚都无法凑出"同一定时器的CH1+CH2"，硬件编码器模式行不通
 *   改为EXTI外部中断+软件M法测速测向（与视频教程方案一致）
 *
 * 【M法测速的设计：为什么只有A相触发中断，B相只是被动读取】
 *   只在A相配置EXTI双边沿中断，B相是普通GPIO输入，
 *   在A相中断发生的瞬间"顺便看一眼"B相当前电平，用来判断方向
 *   这样每次脉冲只触发一次中断（而不是A、B两相各触发一次，
 *   避免重复计数和更复杂的双通道同步处理）
 * ================================================================
 */


/* 新方案（T法，对齐标准代码）*/
static volatile int64_t encoder_l = 0;
static volatile int64_t encoder_r = 0;
static volatile int8_t  direction_l = 1;
static volatile int8_t  direction_r = 1;
static volatile uint64_t t0_l = 0, t1_l = 0;
static volatile uint64_t t0_r = 0, t1_r = 0;

/**
 * ================================================================
 * [驱动层-内部] 编码器GPIO初始化（static）
 * ================================================================
 *
 * 职责：配置四个编码器引脚为上拉输入，解除PB3/PB4的JTAG占用
 *
 * 【为什么用上拉输入(IPU)而不是浮空输入】
 *   编码器输出是开关量电平，上拉输入提供确定的默认高电平，
 *   避免引脚悬空时因为外部干扰产生抖动误触发
 *
 * 【解除JTAG占用的两步，缺一不可】
 *   1. 必须先开启RCC_APB2Periph_AFIO时钟
 *   2. 再调用GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)
 *   这个组合只关闭JTAG的4条额外调试线(包括TDO/NJTRST即PB3/PB4)，
 *   SWD调试用的PA13/14完全不受影响，ST-Link依然能正常连接调试
 * ================================================================
 */
static void Encoder_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);     // 解除JTAG占用前必须先开AFIO时钟

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 释放PB3/PB4，SWD(PA13/14)不受影响

    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;             // 上拉输入，提供确定默认电平
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 只把A相(PB3/PB14)接入EXTI线，B相不需要 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
}

/**
 * ================================================================
 * [驱动层-内部] EXTI中断配置（static）
 * ================================================================
 *
 * 职责：A相(Line3/Line14)双边沿触发中断
 * 双边沿(Rising_Falling) → 上升沿和下降沿都触发，
 * 这样转速不变时，中断频率是脉冲频率的2倍，分辨率翻倍
 * ================================================================
 */
static void Encoder_EXTI_Init(void)
{
    EXTI_InitTypeDef    EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = EXTI_Line3 | EXTI_Line14;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;

    EXTI_Init(&EXTI_InitStructure);
}

/**
 * ================================================================
 * [驱动层-内部] NVIC中断优先级配置（static）
 * ================================================================
 *
 * 职责：使能EXTI3和EXTI15_10两个中断通道
 *
 * 【曾犯的错误】
 *   最初尝试把两个IRQChannel用|合并，企图一次NVIC_Init()调用搞定：
 *   NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn | EXTI15_10_IRQn;  // 错误写法
 *   NVIC_InitTypeDef的IRQChannel字段一次只能指定一个中断通道，
 *   不能像EXTI_Line那样用按位或合并多个
 *   正确做法：同一个结构体变量，分两次调用NVIC_Init()，
 *   每次只改IRQChannel这一个字段，其余字段保持不变复用
 * ================================================================
 */
static void Encoder_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

    NVIC_Init(&NVIC_InitStructure);

    /* 不能和上面合并成一次调用，必须分两次Init */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * ================================================================
 * [驱动层] 编码器初始化（对外Action接口）
 * ================================================================
 */
void Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_EXTI_Init();
    Encoder_NVIC_Init();
}

/**
 * ================================================================
 * [驱动层] M法测速（对外Get接口）
 * ================================================================
 *
 * M法原理：返回"这次调用"和"上次调用"之间，计数值的增量
 * 调用频率固定时（比如Task_Manager里每5ms调用一次），
 * 这个增量天然就正比于转速
 *
 * 【曾犯的错误：类型不一致】
 *   Encoder_Left_Num从int16_t升级为int32_t后（防溢出，见下方说明），
 *   这两个函数内部的Prev_Encoder_Left_Num和差值变量
 *   一度还停留在int16_t，没有跟着同步升级，
 *   导致和源头数据类型不匹配，存在隐式转换风险
 *   统一改为int32_t后类型链条才完全一致
 *
 * 【为什么Prev_xxx要加static】
 *   必须记住"上一次调用时"的计数值才能算差值，
 *   这是static局部变量"跨调用保留状态"的典型应用场景，
 *   和GetData()里的临时变量(用完即焚、不需要static)正好相反
 * ================================================================
 */

float Encoder_Get_L_Speed(void)
{
    __disable_irq();
    int8_t   dir = direction_l;
    uint64_t t0  = t0_l;
    uint64_t t1  = t1_l;
    __enable_irq();

    if(dir == +2 || dir == -2) return 0.0f;

    uint64_t now = System_GetUs();
    float T = (t0 - t1 > now - t0) ?
              (t0 - t1) * 1.0e-6f :
              (now - t0) * 1.0e-6f;

    return (float) dir / T / 22.0f / (30613.0f / 1500.0f) * 6.2831853f;
}

float Encoder_Get_R_Speed(void)
{
    __disable_irq();
    int8_t   dir = direction_r;
    uint64_t t0  = t0_r;
    uint64_t t1  = t1_r;
    __enable_irq();

    if(dir == +2 || dir == -2) return 0.0f;

    uint64_t now = System_GetUs();
    float T = (t0 - t1 > now - t0) ?
              (t0 - t1) * 1.0e-6f :
              (now - t0) * 1.0e-6f;

    return (float) dir / T / 22.0f / (30613.0f / 1500.0f) * 6.2831853f;
}



/**
 * ================================================================
 * [驱动层] 获取累计位置（对外Get接口）
 * ================================================================
 *
 * 【为什么从int16_t升级为int32_t，曾经的溢出风险计算】
 *   假设典型脉冲率：1000脉冲/转 × 200RPM ≈ 200,000脉冲/分钟
 *   int16_t最大值32767 → 32767/200000 ≈ 0.16分钟 ≈ 10秒就会溢出
 *   int32_t最大值约21亿 → 21亿/200000 ≈ 10500分钟 ≈ 7天才会溢出
 *   平衡车单次运行可能持续几分钟到几十分钟，int16_t完全不够用，
 *   必须升级为int32_t（这个计算过程本身也是一次有效的工程判断训练：
 *   不是"凭感觉觉得可能溢出"，而是真正估算脉冲率、算出具体溢出时间）
 * ================================================================
 */

int32_t Encoder_Get_L_Position(void)
{
    return (int32_t)encoder_l;
}

int32_t Encoder_Get_R_Position(void)
{
    return (int32_t)encoder_r;
}

/**
 * ================================================================
 * [驱动层] EXTI中断服务函数 —— 方向判断与计数
 * ================================================================
 *
 * 方向判断真值表（A相中断触发瞬间，读取B相电平判断转向）：
 *   A上升沿 + B低电平 → +1
 *   A上升沿 + B高电平 → -1
 *   A下降沿 + B高电平 → +1
 *   A下降沿 + B低电平 → -1
 *   四种边沿组合全部覆盖，缺一种就会在某个转动方向上漏计数
 *
 * 【EXTI_ClearITPendingBit()的位置】
 *   必须放在if(EXTI_GetFlagStatus()==SET)判断内部，
 *   只清除"确实发生过"的中断标志，避免误清掉其他未处理的状态
 *
 * 【待验证 / 已知遗留问题】
 *   方向正负号的正确性尚未经过实际转动轮子+串口打印验证
 *   如果实测发现方向相反，修复方式是把上面四个if判断里的++/--对调
 *   （这和电机镜像安装时处理>=/<=对调是同一类"驱动层内部
 *   吸收硬件差异、算法层无感知"的封装思路）
 * ================================================================
 */
void EXTI3_IRQHandler(void)
{
    if(EXTI_GetFlagStatus(EXTI_Line3) == SET)  // 加这一行
    {
        EXTI_ClearFlag(EXTI_Line3);
        t1_r = t0_r;
        t0_r = System_GetUs();

        uint8_t a = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
        uint8_t b = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);

        if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET))
        {
            encoder_r++;
            direction_r = (direction_r < 0) ? +2 : 1;
        }
        else
        {
            encoder_r--;
            direction_r = (direction_r > 0) ? -2 : -1;
        }
        EXTI_ClearFlag(EXTI_Line3);
    }
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetFlagStatus(EXTI_Line14) == SET)
    {
        EXTI_ClearFlag(EXTI_Line14);

        t1_l = t0_l;
        t0_l = System_GetUs();

        uint8_t a = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
        uint8_t b = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);

        if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET))
        {
            encoder_l--;
            direction_l = (direction_l > 0) ? -2 : -1;
        }
        else
        {
            encoder_l++;
            direction_l = (direction_l < 0) ? +2 : 1;
        }
    }
}


/* ================================================================
 * 【备用方案】M法测速（已被T法替代，保留供参考）
 * ================================================================
 *
 * M法原理：返回固定时间窗口内的脉冲增量，正比于转速
 * 局限：低速时每5ms只有0-1个脉冲，速度读数时有时无
 *
 * 使用M法需要的静态变量（替换T法变量）：
 *   static volatile int32_t Encoder_Left_Num  = 0;
 *   static volatile int32_t Encoder_Right_Num = 0;
 *
 * int32_t Encoder_Get_L_Speed(void)
 * {
 *     static int32_t Prev_Encoder_Left_Num;
 *     __disable_irq();
 *     int32_t current = Encoder_Left_Num;
 *     __enable_irq();
 *     int32_t diff = current - Prev_Encoder_Left_Num;
 *     Prev_Encoder_Left_Num = current;
 *     return diff;
 * }
 *
 * int32_t Encoder_Get_R_Speed(void)
 * {
 *     static int32_t Prev_Encoder_Right_Num;
 *     __disable_irq();
 *     int32_t current = Encoder_Right_Num;
 *     __enable_irq();
 *     int32_t diff = current - Prev_Encoder_Right_Num;
 *     Prev_Encoder_Right_Num = current;
 *     return diff;
 * }
 *
 * M法中断处理（右编码器EXTI3）：
 * void EXTI3_IRQHandler(void)
 * {
 *     if(EXTI_GetFlagStatus(EXTI_Line3) == SET)
 *     {
 *         if(PB3==SET   && PB4==RESET) Encoder_Right_Num++;
 *         if(PB3==SET   && PB4==SET)   Encoder_Right_Num--;
 *         if(PB3==RESET && PB4==SET)   Encoder_Right_Num++;
 *         if(PB3==RESET && PB4==RESET) Encoder_Right_Num--;
 *         EXTI_ClearITPendingBit(EXTI_Line3);
 *     }
 * }
 *
 * M法中断处理（左编码器EXTI15_10）：
 * void EXTI15_10_IRQHandler(void)
 * {
 *     if(EXTI_GetFlagStatus(EXTI_Line14) == SET)
 *     {
 *         if(PB14==SET   && PB15==SET)   Encoder_Left_Num++;
 *         if(PB14==SET   && PB15==RESET) Encoder_Left_Num--;
 *         if(PB14==RESET && PB15==SET)   Encoder_Left_Num++;
 *         if(PB14==RESET && PB15==RESET) Encoder_Left_Num--;
 *         EXTI_ClearITPendingBit(EXTI_Line14);
 *     }
 * }
 *
 * M法时EncoderFeedback.c的换算（需同时修改）：
 * #define ENCODER_LINE_COUNTE  22
 * #define REDUCTION_RATIO      (30613.0f / 1500.0f)
 * #define FEEDBACK_DT          0.005f
 *
 * Speed_Output.Left_Speed = (float)Encoder_Get_L_Speed()
 *     / (ENCODER_LINE_COUNTE * REDUCTION_RATIO)
 *     * (2 * 3.1415926f / FEEDBACK_DT);
 * ================================================================ */
 

