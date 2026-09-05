#include "stm32f10x.h"


/**
 * TB6612FNG H桥控制真值表
 * ================================================================
 * IN1   IN2   PWM   STBY  OUT1  OUT2  模式
 * ----------------------------------------------------------------
 * H     H     H/L   H     L     L     Short brake（快速制动）
 * L     H     H     H     L     H     CCW（反转）
 * L     H     L     H     L     L     Short brake
 * H     L     H     H     H     L     CW（正转）
 * H     L     L     H     L     L     Short brake
 * L     L     H     H     OFF   OFF   Stop（自由滑行，高阻态）
 * H/L   H/L   H/L   L     OFF   OFF   Standby（待机）
 * ================================================================
 *
 * 平衡车使用策略：
 *   speed > 0 → IN1=H, IN2=L, PWM=占空比  → CW正转
 *   speed < 0 → IN1=L, IN2=H, PWM=占空比  → CCW反转
 *   speed = 0 → IN1=H, IN2=H, PWM=任意    → Short brake快速制动
 *
 * 【为什么用Short brake而不是Stop】
 *   Stop（高阻态）→ 电机自由滑行，靠惯性停下，响应慢
 *   Short brake  → 电机两端短接，快速制动，平衡车需要快速响应
 *
 * 【用>=和<=处理speed=0的制动】
 *   IN1: speed >= 0 ? Bit_SET : Bit_RESET → 正数和0都是H
 *   IN2: speed <= 0 ? Bit_SET : Bit_RESET → 负数和0都是H
 *   speed=0时IN1=H, IN2=H → Short brake，不需要单独if分支
 *
 * 注意：
 *   STBY必须保持H（PA1已在Motor_GPIO_Init中拉高）
 *   STBY=L → 所有输出高阻态（Standby），电机不工作
 * ================================================================
 */

/**
 * [驱动层-内部] 电机方向控制GPIO初始化
 * 【STBY引脚的作用】
 *   TB6612_STBY = PA1，初始化后必须拉高
 *   否则TB6612处于Standby状态，所有输出高阻态，电机不响应任何控制
 */
static void Motor_GPIO_Init(void)
{
    /* GPIOA和GPIOB均挂载APB2总线 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;  // 只声明一次，后续直接复用

    /* PA1(STBY) + PA9(L_IN1) + PA10(L_IN2) 合并一次Init
     * 三个引脚模式相同，可以用|合并 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PB5(R_IN1) + PB7(R_IN2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 使能TB6612，STBY拉高，否则电机不工作 */
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
}

/**
 * ================================================================
 * [驱动层-内部] 左电机PWM初始化（TIM1_CH1，PA8）（static）
 * ================================================================
 *
 * TIM1是高级定时器，比通用定时器多以下参数：
 *   TIM_RepetitionCounter → 重复计数器，普通PWM填0
 *   TIM_OCNxxx参数       → 互补输出相关，不用互补时填Disable/Reset
 *   TIM_CtrlPWMOutputs() → 必须调用才能使能主输出，通用定时器不需要
 *
 * PWM参数推导：
 *   系统时钟72MHz / 预分频72 / ARR100 = 10kHz
 *   ARR=99 → CCR范围0~99 → 占空比精度1%
 *
 * 【曾犯的错误】
 *   1. TIM_OutputNState用了TIM_OutputState_Disable（规则组枚举）
 *      正确：TIM_OutputNState_Disable（互补通道专属枚举）
 *
 *   2. 忘记调用TIM_CtrlPWMOutputs(TIM1, ENABLE)
 *      TIM1高级定时器特有，不调用则PWM引脚无输出
 *
 * 【曾有的疑问】
 *   Q: TIM_ClockDivision是什么？
 *   A: 不是系统时钟分频，而是输入捕获滤波器和死区时间的采样时钟
 *      对PWM输出没有影响，填TIM_CKD_DIV1保持默认即可
 *      涉及输入捕获滤波或互补PWM死区时才需要调整
 * ================================================================
 */
static void Motor_Left_PWM_Init(void)
{
    /* TIM1挂载APB2总线（高速），PA8也在APB2 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA8配置为复用推挽输出
     * TIM1外设控制PA8输出PWM波形，CPU不参与 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 时基配置：72MHz/72/100 = 10kHz PWM频率 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode        = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period             = 100 - 1;   // ARR=99
    TIM_TimeBaseInitStructure.TIM_Prescaler          = 72 - 1;    // 72MHz→1MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;         // TIM1专属，普通PWM填0
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    /* 输出比较配置：PWM1模式，初始占空比0 */
    TIM_OCInitTypeDef TIM_OutputCompareInitStructure;
    TIM_OutputCompareInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OutputCompareInitStructure.TIM_Pulse        = 0;                        // CCR初始值，运行时由Motor_Speed_Set更新
    TIM_OutputCompareInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OutputCompareInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OutputCompareInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    /* 互补通道：不使用，全部禁用 */
    TIM_OutputCompareInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OutputCompareInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OutputCompareInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OutputCompareInitStructure);
    /*  两个 PWM 初始化函数都没有启用 OC 预装载寄存器。不启用时，TIM_SetCompare1 的更新立即生效（可能在 PWM
  周期中途），导致当前脉冲被截断或延长，产生毛刺电流。*/
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    /* TIM1高级定时器必须调用，否则PWM无输出 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * [驱动层-内部] 右电机PWM初始化（TIM4_CH1，PB6）（static）
 *
 * TIM4是通用定时器，比TIM1简单：
 *   无TIM_RepetitionCounter
 *   无互补输出参数
 *   无需TIM_CtrlPWMOutputs()
 */
static void Motor_Right_PWM_Init(void)
{
    /* TIM4挂载APB1总线（低速），PB6在APB2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        = 100 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 72 - 1;
    /* 注意：TIM4是通用定时器，没有TIM_RepetitionCounter */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OutputCompareInitStructure;
    TIM_OutputCompareInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OutputCompareInitStructure.TIM_Pulse       = 0;
    TIM_OutputCompareInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OutputCompareInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OutputCompareInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    /* 注意：TIM4没有互补输出，不需要OCN相关参数 */
    TIM_OC1Init(TIM4, &TIM_OutputCompareInitStructure);
    /*  两个 PWM 初始化函数都没有启用 OC 预装载寄存器。不启用时，TIM_SetCompare1 的更新立即生效（可能在 PWM
  周期中途），导致当前脉冲被截断或延长，产生毛刺电流。*/
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    /* 注意：TIM4不需要TIM_CtrlPWMOutputs() */
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * [驱动层] 双电机初始化（对外Action接口）
 *
 * 职责：封装三个内部初始化函数，对外暴露统一入口
 * 调用方只需要调用这一个函数，不需要知道内部有三个步骤
 *
 * 初始化顺序：
 *   1. GPIO先初始化 → 确保STBY拉高，TB6612使能
 *   2. 左电机PWM
 *   3. 右电机PWM
 *
 * 【曾犯的错误】
 *   把Double_Motors_Init()放在文件最上面，但它调用的三个函数
 *   定义在后面，编译器从上往下读，看到调用时还不认识这些函数。
 *   解决：把Double_Motors_Init()移到三个内部函数后面
 */
void Double_Motors_Init(void)
{
    Motor_GPIO_Init();
    Motor_Left_PWM_Init();
    Motor_Right_PWM_Init();
}

/**
 * ================================================================
 * [驱动层] 设置双电机速度（对外Set接口，算法层唯一调用点）
 * ================================================================
 *
 * 参数：
 *   Left_Speed  → 左电机速度，范围-100~+100
 *   Right_Speed → 右电机速度，范围-100~+100
 *   正值=前进方向，负值=后退方向，0=制动
 *
 * 内部映射：
 *   |speed| → CCR值（0~99）→ PWM占空比（0%~99%）→ 电机转速
 *   speed符号 → IN1/IN2电平 → 电机转向
 *
 * 【镜像安装的处理】
 *   左右电机镜像安装，相同speed值对应相反的物理转向
 *   这个细节封装在驱动层，算法层无感知：
 *     Motor_Speed_Set(100, 100) → 两轮同向前进（内部自动处理镜像）
 *   如果不处理镜像，算法层需要Motor_Speed_Set(100, -100)才能前进
 *   封装后算法层逻辑更直观，符合五层塔接口原则
 *
 * 【防御性编程：限幅】
 *   CCR最大值99（ARR=99），超出范围会溢出
 *   算法层PID输出可能超出范围，驱动层必须自己保护
 *   原则：接口要对输入做合法性检查，不能假设调用方传合法值
 *
 * 【uint16_t PWM值需要取绝对值】
 *   Left_Speed可能是负数，直接赋给uint16_t会溢出
 *   用三元运算符取绝对值：(x < 0 ? -x : x)
 *
 * 【用>=和<=同时处理0的制动状态】
 *   不需要单独处理speed=0的情况：
 *   IN1: speed >= 0 → H（正数和0都给H）
 *   IN2: speed <= 0 → H（负数和0都给H）
 *   speed=0时 IN1=H, IN2=H → Short brake自动成立
 *
 * 【曾有的疑问】
 *   Q: 真实速度和占空比有什么关系？
 *   A: 现在是开环控制，占空比≈速度的百分比
 *      等编码器和PID完成后升级为闭环：
 *      现在：speed = PWM占空比（开环）
 *      以后：speed = 目标转速rpm，PID计算后转换成CCR（闭环）
 *      接口不变，算法层调用方式完全相同
 *
 *   Q: 为什么叫SetSpeed而不是SetPWM？
 *   A: 接口语义应该面向业务（速度），不面向实现（PWM）
 *      算法层不需要知道底层用PWM实现，只需要知道"设置速度"
 * ================================================================
 */
void Motor_Speed_Set(int16_t Left_Speed, int16_t Right_Speed)
{
    /* 防御性编程：限幅，防止超出PWM范围 */
    if (Left_Speed  >  100) Left_Speed  =  100;
    if (Left_Speed  < -100) Left_Speed  = -100;
    if (Right_Speed >  100) Right_Speed =  100;
    if (Right_Speed < -100) Right_Speed = -100;
    if(Left_Speed == 0)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_9,  Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_SET);
    }
    else
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_9,  Left_Speed > 0 ? Bit_RESET : Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_10, Left_Speed > 0 ? Bit_SET   : Bit_RESET);
    }
    
    if(Right_Speed == 0)
    {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
    }
    else
    {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5, Right_Speed > 0 ? Bit_SET   : Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_7, Right_Speed > 0 ? Bit_RESET : Bit_SET);
    }

    /* 左电机PWM更新：取绝对值转换为CCR */
    uint16_t Left_PWM  = (uint16_t)(Left_Speed  < 0 ? -Left_Speed  : Left_Speed)  * 99 / 100;
    TIM_SetCompare1(TIM1, Left_PWM);

    /* 右电机PWM更新 */
    uint16_t Right_PWM = (uint16_t)(Right_Speed < 0 ? -Right_Speed : Right_Speed) * 99 / 100;
    TIM_SetCompare1(TIM4, Right_PWM);
}



