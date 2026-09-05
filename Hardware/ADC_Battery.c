#include "stm32f10x.h"


static volatile uint16_t Voltage_Anal;   // Raw ADC Data, 0~4095
static volatile uint8_t  JEOC_Status;    // New Data Flag, 1 means new data arrived

static volatile float Vbat = 0.0f;


/* 
 * [Feedback processing layer interface] Get raw ADC value
 * Purpose: For monitoring-layer threshold checks — 
 * comparing raw ADC values directly is more efficient and avoids floating-point math
 * Threshold reference          (derived from hardware parameters, voltage-divider ratio = 3.3/8.4):
 *  3399 → Battery 70%~100%     (three LEDs on)
 *  3193 → Battery 40%~70%      (two LEDs on)
 *  2990 → Battery 10%~40%      (one LED on)
 * ≤2990 → Battery <10%         (all off, needs charging)
 * */

uint16_t Battery_GetVoltageAnal(void)
{
    return Voltage_Anal;
}

/* 
 * [Feedback processing layer interface] Get the converted actual voltage value
 *
 * Conversion formula derivation:
 *   V_bat = ADC value × (VREF+ / 4095) × (1 / divider ratio)
 *         = ADC value × (3.3 / 4095) × (8.4 / 3.3)
 *         = ADC value × (8.4 / 4095)
 * */

float Battery_GetVoltage(void)
{
    return Vbat + 1.4f;     //Adding the f suffix explicitly makes them float, which is faster.
}

/* 
 * [Scheduler layer interface] Check whether new ADC data is available
 * */
uint8_t Battery_GetFlag(void)
{
    uint8_t temp = JEOC_Status;
    JEOC_Status = 0;          // Cleared after being read, to prevent duplicate consumption.
    return temp;
}

/**
 * [Driver layer] ADC battery voltage acquisition - Timer initialization
 * Configure TIM2 as a 10ms periodic timer, triggering ADC injected-group sampling via TRGO hardware trigger.
 */
void ADC_Battery_TIM_Init(void)
{
    /* [1] Enable TIM2 clock
     * TIM2 is on the APB1 bus (low-speed, max 36MHz; 
     * after frequency multiplication, 72MHz is supplied to the timer) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* [2] Configure time-base parameters */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 72 - 1;               // 72MHz->1MHz, 1μs per count
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         // keep default
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;   // Counting up, 0→ARR overflow
    TIM_TimeBaseInitStructure.TIM_Period        = 10000 - 1;            // 10000μs = 10ms overflow

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* [3] Configure the TRGO output source as the Update event
     * Each time the counter overflows (Update event), TRGO issues a trigger signal to the ADC*/
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

    /* [4] 使能TIM2，开始计数
     * 使能后计数器从0开始，每10ms产生一次Update事件和TRGO信号 */
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * ================================================================
 * [驱动层] ADC电池电压采集 - ADC初始化
 * ================================================================
 *
 * 职责：
 *   配置ADC1注入组，采集PB0(ADC1_CH8)的电池分压信号。
 *   由TIM2_TRGO每10ms硬件触发一次，转换完成触发JEOC中断。
 *
 * 硬件约束（来自原理图）：
 *   VBAT_SENSE → PB0 → ADC1_CH8
 *   分压网络：R6=5.1kΩ（上），R14=3.3kΩ（下）
 *   分压比 = R14/(R6+R14) = 3.3/8.4
 *   VREF+ = 3.3V（接3V3电源轨，由MT2492提供）
 *   → 满电8.4V经分压后 = 3.3V = ADC满量程4095
 *
 * 信号流（五层塔驱动层的核心链路）：
 *   TIM2_TRGO(10ms) → ADC1注入组采样CH8(PB0)
 *   → 转换完成 → 结果存入JDR1 → 触发JEOC中断
 *   → 中断读取JDR1 → 存入Voltage_Anal → 举JEOC_Status旗
 *   → 主循环看到旗子 → 比较阈值 → 控制LED
 *
 * 外设初始化的通用顺序（重要，不能随意颠倒）：
 *   1. 开时钟
 *   2. 配GPIO
 *   3. 配外设参数（填结构体→Init）
 *   4. 使能外设
 *   5. 特殊步骤（ADC需要校准）
 *   6. 使能触发和中断
 *
 */
void ADC_Battery_Init(void)
{
    /* [1] 开启ADC1和GPIOB时钟（均挂载APB2总线） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* [2] PB0配置为模拟输入
     * 模拟输入不需要配置Speed，GPIO_Mode_AIN下该参数无意义 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* [3] 配置注入组通道（注入组专属函数，不在ADC_InitTypeDef里）
     * 通道8对应PB0，注入序列第1位，采样时间7.5个周期
     * 信号源阻抗低（R6+R14并联≈2kΩ），采样时间不需要太长 */
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_1Cycles5);

    /* [4] 配置注入组外部触发源为TIM2_TRGO（注入组专属函数）
     * TIM2_TRGO只在注入组触发源列表里，规则组列表里没有它
     * 这是选择注入组而非规则组的根本原因（硬件约束决定） */
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T2_TRGO);

    /* [5] 配置ADC基本参数（规则组参数，对注入组无影响）
     * ExternalTrigConv设为None，明确表示规则组不使用外部触发 */
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                     // 单次转换，由TIM触发，不自动连续
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;         // 右对齐，结果在低12位，直接读0~4095
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;   // 规则组不用外部触发
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;        // 独立模式，ADC1独立工作
    ADC_InitStructure.ADC_NbrOfChannel       = 1;                           // 规则组转换1个通道
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;                     // 单通道不需要扫描模式
    ADC_Init(ADC1, &ADC_InitStructure);                                     // 必须调用Init才能生效！

    /* [6] 使能ADC1
     * 必须先使能，后面的校准和触发配置才有效 */
    ADC_Cmd(ADC1, ENABLE);

    /* [7] ADC自校准
     * 消除内部电容误差，提高转换精度，上电必须执行一次
     * SET=校准进行中（继续等），RESET=校准完成（退出循环）
     * 口诀：等到"好了"才动手 */
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) == SET);

    /* [8] 使能注入组外部触发
     * 必须在ADC_Cmd(ENABLE)之后调用才有效
     * 使能后TIM2_TRGO信号才能真正触发ADC开始转换 */
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);

    /* [9] 使能JEOC中断
     * 注入组转换完成时触发中断，在IRQHandler里读取JDR1结果 */
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
}

/**
 * ================================================================
 * [驱动层] NVIC中断控制器配置
 * ================================================================
 *
 * NVIC负责决定多个中断同时发生时谁先执行、谁能打断谁。
 *
 * 优先级分组（在main.c里全局配置一次）：
 *   NVIC_PriorityGroup_2 → 抢占优先级2位(0~3)，子优先级2位(0~3)
 *
 * 本模块优先级：抢占2，子优先级2
 *   低于TIM3时基（抢占0），保证控制节拍不被打断
 *   ADC电压采样可以稍慢，偶尔被打断没有影响
 *
 */
void NVIC_Battery_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC1_2_IRQn; // ADC1和ADC2共用中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;           // 低于TIM3时基(0)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStructure);
}


/**
 * ================================================================
 * [驱动层] ADC注入组转换完成中断处理函数
 * ================================================================
 *
 * 触发条件：ADC1注入组转换完成（JEOC标志位置位）
**/
void ADC1_2_IRQHandler(void)
{
    if(ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == SET)
    {
        /* 读取原始ADC值 */
        Voltage_Anal = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);

        /* 直接换算成浮点电压（对齐标准代码） */
        Vbat = Voltage_Anal / 4095.0f * 8.4f;

        /* 举旗通知 */
        JEOC_Status = 1;

        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}


