#include "stm32f10x.h"
#include "SYSTEM_TIM.h"


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
       freeing up PB3/PB4 — SWD (PA13/14) is unaffected*/
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
 * [Driver layer] Improved T-method speed measurement (external Get interface)
 * ================================================================
 *
 * [T-method principle]
 *   Unlike the M-method (counting pulse increments over a fixed time
 *   window), the T-method records the microsecond-level timestamp of
 *   each pulse edge and computes instantaneous speed from the inverse
 *   of the period (1/T) — noticeably more accurate than the M-method
 *   at low speed, where pulses are sparse
 *
 * [The improvement: why not just use T = t0 - t1]
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
 * [Returning 0 when dir == ±2: discarding the edge right after a
 *  direction reversal]
 *   ±2 is a temporary marker set inside the EXTI interrupt meaning
 *   "this edge's direction is opposite to the previous one." Right
 *   after a reversal, the interval t0-t1 spans both the forward and
 *   reverse motion, so the T computed from it is not physically
 *   meaningful — it's discarded and 0 is returned instead. Normal
 *   computation resumes once the next edge confirms a steady
 *   direction (+1/-1)
 *
 * [Atomic protection via __disable_irq()/__enable_irq()]
 *   dir/t0/t1 are each updated by the EXTI interrupt, so they must be
 *   read as a group under protection — otherwise a half-updated
 *   combination could be read (e.g. t0 already new but t1 still old)
 *
 * Return value: wheel angular velocity in rad/s, sign indicates direction
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
 * 【中文对照版 —— 仅供review，确认后可删除】
 * Encoder_Get_L_Speed() / Encoder_Get_R_Speed() 函数头注释：
 * ================================================================
 *
 * [驱动层] 改进T法测速（对外Get接口）
 *
 * 【T法原理】
 *   不同于M法"固定时间窗口内数脉冲增量"，T法记录相邻两次脉冲边沿的
 *   微秒级时间戳，用周期的倒数(1/T)直接算出瞬时转速，
 *   低速、脉冲稀疏时精度明显优于M法
 *
 * 【改进点：为什么不直接用T = t0 - t1】
 *   纯T法只用"上一个完整周期"(t0-t1)算速度，
 *   但如果车轮突然减速甚至停下，最后一个周期结束后不再有新脉冲，
 *   T会一直停留在旧值，速度读数"卡"在减速前的水平，无法归零
 *   解决办法：额外比较"从上次边沿到现在"已经等待的时间(now-t0)，
 *   取(t0-t1)和(now-t0)中较大的一个作为T——
 *   车轮越慢，now-t0增长得越大，算出的速度就越小，
 *   读数能连续收敛到0，而不是卡死在旧值上
 *
 * 【dir == ±2 时直接返回0：方向刚翻转的边沿丢弃】
 *   ±2是EXTI中断里的临时标记，表示"这一次边沿和上一次方向相反"
 *   方向刚翻转时，t0-t1这个区间横跨了正转和反转两段时间，
 *   算出来的T没有物理意义，所以直接丢弃返回0，
 *   等下一个边沿到来、方向稳定为+1/-1后再正常计算
 *
 * 【__disable_irq()/__enable_irq()原子保护】
 *   dir/t0/t1三个变量分别由EXTI中断更新，读取时必须成组保护，
 *   否则可能读到"更新到一半"的组合（如t0已是新值，t1还是旧值）
 *
 * 返回值：轮子角速度，单位rad/s，符号表示方向
 * ================================================================ */

