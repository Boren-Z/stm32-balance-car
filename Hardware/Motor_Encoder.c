#include "stm32f10x.h"
#include "SYSTEM_TIM.h"

/**
 * [Driver layer] Improved T-method speed measurement 
 *
 * T-method principle
 *   Unlike the M-method (counting pulse increments over a fixed time
 *   window), the T-method records the microsecond-level timestamp of
 *   each pulse edge and computes instantaneous speed from the inverse
 *   of the period (1/T) — noticeably more accurate than the M-method
 *   at low speed, where pulses are sparse
 *
 * The improvement: why not just use T = t0 - t1
 *   A plain T-method only uses the last complete period (t0-t1) to
 *   compute speed. But if the wheel suddenly slows down or stops, no
 *   new edge arrives after the last period ends, so T stays frozen at
 *   its old value and the speed reading gets "stuck" at the
 *   pre-deceleration level instead of dropping toward zero
 *   Fix: also compare the time already waited since the last edge
 *   (now-t0), and use whichever of (t0-t1) and (now-t0) is larger as T
 *   — the slower the wheel gets, the larger now-t0 grows, so the
 *   computed speed keeps shrinking and converges continuously to 0,
 *   instead of staying stuck at a stale value
 *
 * Returning 0 when dir == ±2: discarding the edge right after a
 *  direction reversal
 *   ±2 is a temporary marker set inside the EXTI interrupt meaning
 *   "this edge's direction is opposite to the previous one." Right
 *   after a reversal, the interval t0-t1 spans both the forward and
 *   reverse motion, so the T computed from it is not physically
 *   meaningful — it's discarded and 0 is returned instead. Normal
 *   computation resumes once the next edge confirms a steady
 *   direction (+1/-1)
 *
 * Atomic protection via __disable_irq()/__enable_irq()
 *   dir/t0/t1 are each updated by the EXTI interrupt, so they must be
 *   read as a group under protection — otherwise a half-updated
 *   combination could be read (e.g. t0 already new but t1 still old)
 *
 * Return value: wheel angular velocity in rad/s, sign indicates direction
 */

static volatile int64_t encoder_l = 0;
static volatile int64_t encoder_r = 0;
static volatile int8_t  direction_l = 1;
static volatile int8_t  direction_r = 1;
static volatile uint64_t t0_l = 0, t1_l = 0;
static volatile uint64_t t0_r = 0, t1_r = 0;

/* [Driver layer] Encoder GPIO initialization */
static void Encoder_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    // The AFIO clock must be enabled before releasing the JTAG-occupied pins
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    /* Disable the JTAG's 4 extra debug lines (including TDO/NJTRST, i.e. PB3/PB4), 
       freeing up PB3/PB4 — SWD (PA13/14) is unaffected
    */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 

    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Pull-up input, providing a definite default level
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Only phase A (PB3/PB14) is connected to the EXTI line — phase B doesn't need it */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
}

/**
 * [Driver layer - internal] EXTI interrupt configuration 
 *
 * dual-edge-triggered interrupt on phase A (Line3/Line14)
 * Dual edge (Rising_Falling) → both rising and falling edges trigger,
 * so at a constant speed, the interrupt frequency is 2× the pulse
 * frequency, doubling the resolution
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
 * [Driver layer] NVIC interrupt priority configuration
 */
static void Encoder_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * [Driver layer] Encoder initialization (external interface)
 */
void Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_EXTI_Init();
    Encoder_NVIC_Init();
}


float Encoder_Get_L_Speed(void)
{
    __disable_irq();
    int8_t   dir = direction_l;
    uint64_t t0  = t0_l;
    uint64_t t1  = t1_l;
    __enable_irq();

    if(dir == +2 || dir == -2) return 0.0f;

    uint64_t now = System_GetUs();
    float T = (t0 - t1 > now - t0) ?  (t0 - t1) * 1.0e-6f :  (now - t0) * 1.0e-6f;

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
    float T = (t0 - t1 > now - t0) ?  (t0 - t1) * 1.0e-6f :  (now - t0) * 1.0e-6f;

    return (float) dir / T / 22.0f / (30613.0f / 1500.0f) * 6.2831853f;
}



/* [Driver layer] Get accumulated position (external Get interface) */

int64_t Encoder_Get_L_Position(void)
{
    /** Upgraded from int16_t to int64_t. Assuming a typical pulse rate:
      *  1000 pulses/rev × 200RPM ≈ 200,000 pulses/minute,
      *  int16_t max value 32767 → 32767/200000 ≈ 0.16 min ≈ overflows in about 10 seconds
      */
    return (int64_t)encoder_l;
}

int64_t Encoder_Get_R_Position(void)
{
    return (int64_t)encoder_r;
}

/**
 * [Driver layer] EXTI interrupt service routine — direction detection and counting
 *
 * Direction truth table (at the instant phase A triggers the interrupt,
 * read phase B's level to determine rotation direction):
 *   A rising edge  + B low  → +1
 *   A rising edge  + B high → -1
 *   A falling edge + B high → +1
 *   A falling edge + B low  → -1
 *   All four edge combinations are covered — missing any one would
 *   miss counts in one rotation direction
 *
 * EXTI_ClearFlag() must be placed inside the if(EXTI_GetFlagStatus()==SET) check, 
 * so it only clears the flag that "actually occurred,"
 * avoiding accidentally clearing other unhandled pending state
 *
 */

void EXTI3_IRQHandler(void)
{
    if(EXTI_GetFlagStatus(EXTI_Line3) == SET)  
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




