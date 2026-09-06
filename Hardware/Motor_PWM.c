#include "stm32f10x.h"



/*[Driver layer] Motor direction control GPIO initialization*/
static void Motor_GPIO_Init(void)
{
    /* GPIOA and GPIOB are both mounted on the APB2 bus*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;  

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PB5(R_IN1) + PB7(R_IN2) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Enable the TB6612, pull STBY high — otherwise the motor won't work */
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
}

/**
 * [Driver layer - internal] Left motor PWM initialization (TIM1_CH1, PA8) (static)
 * PWM parameter derivation:
 * System clock 72MHz / prescaler 72 / ARR 100 = 10kHz
 * ARR=99 → CCR range 0~99 → duty-cycle resolution 1%
 * [10kHz was chosen for the PWM frequency]
 * Too low (<1kHz) -> motor buzzes audibly, large current ripple, low efficiency
 * Too high (>100kHz) -> TB6612 switching losses are large, severe heating
 * 10kHz -> above the human hearing limit (near the 20kHz boundary), switching losses acceptable
 *  control period is far below the PWM frequency, motor response is fast enough

 */
static void Motor_Left_PWM_Init(void)
{
    /* TIM1, PA8 mounted on the APB2 bus — also on APB2 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA8 configured as alternate-function push-pull output — 
     * TIM1 peripheral drives PA8 to output the PWM waveform, CPU is not involved */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Time-base configuration: 72MHz/72/100 = 10kHz PWM frequency */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode        = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period             = 100 - 1;   // ARR = 99
    TIM_TimeBaseInitStructure.TIM_Prescaler          = 72 - 1;    // 72MHz -> 1MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;         // TIM1-specific, fill with 0 for normal PWM
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    /* Output compare configuration: PWM1 mode, initial duty cycle 0 */
    TIM_OCInitTypeDef TIM_OutputCompareInitStructure;
    TIM_OutputCompareInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OutputCompareInitStructure.TIM_Pulse        = 0;                        
    TIM_OutputCompareInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OutputCompareInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OutputCompareInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    /* Complementary channel: not used, fully disabled */
    TIM_OutputCompareInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OutputCompareInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OutputCompareInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OutputCompareInitStructure);

    /* Both PWM channels enable the OC preload register. Without it, 
       an update from TIM_SetCompare1 takes effect immediately (possibly mid PWM-cycle), 
       truncating or extending the current pulse and causing a current glitch. */
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* TIM1, being an advanced-control timer, must call this */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/**
 * [Driver layer] Right motor PWM initialization (TIM4_CH1, PB6)
 */
static void Motor_Right_PWM_Init(void)
{
    /* TIM4 is mounted on the APB1 bus, PB6 is on APB2 */
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

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OutputCompareInitStructure;
    TIM_OutputCompareInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OutputCompareInitStructure.TIM_Pulse       = 0;
    TIM_OutputCompareInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OutputCompareInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OutputCompareInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    /* TIM4 has no complementary output, so the OCN-related parameters aren't needed */
    TIM_OC1Init(TIM4, &TIM_OutputCompareInitStructure);
    
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_Cmd(TIM4, ENABLE);
}

/**
 * [Driver layer] Dual-motor initialization (external interface)
 * Wraps the three internal initialization functions, exposing a single unified entry point. 
 * Callers only need to call this one function, without needing to know there are three internal steps. 
 * 
 **/
void Double_Motors_Init(void)
{
    Motor_GPIO_Init();
    Motor_Left_PWM_Init();
    Motor_Right_PWM_Init();
}

/**
 * [Driver layer] Set dual-motor speed (external Set interface, the sole call site from the algorithm layer)
 */
void Motor_Speed_Set(int16_t Left_Speed, int16_t Right_Speed)
{
    /* Defensive programming: clamp to prevent exceeding the PWM range. Parameters:
     * Left_Speed  → Left motor speed, range -100~+100
     * Right_Speed → Right motor speed, range -100~+100
     * Positive = forward direction, negative = reverse direction, 0 = brake
     * The algorithm layer's PID output may exceed the range — the driver layer must protect itself
     * Principle: an interface must validate its inputs — never assume the caller passes legal values
     */
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

    /*
     *   Internal mapping:
     *   |speed| → CCR value (0~99) → PWM duty cycle (0%~99%) → motor speed
     *   sign of speed → IN1/IN2 levels → motor direction
     */
    /* Left motor PWM update: take the absolute value and convert to CCR */
    uint16_t Left_PWM  = (uint16_t)(Left_Speed  < 0 ? -Left_Speed  : Left_Speed)  * 99 / 100;
    TIM_SetCompare1(TIM1, Left_PWM);

    /* Right motor PWM update */
    uint16_t Right_PWM = (uint16_t)(Right_Speed < 0 ? -Right_Speed : Right_Speed) * 99 / 100;
    TIM_SetCompare1(TIM4, Right_PWM);
}



