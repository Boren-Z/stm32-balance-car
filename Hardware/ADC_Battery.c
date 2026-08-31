#include "stm32f10x.h"

/**
 * ================================================================
 * 【五层塔定位】驱动层 - ADC电池电压采集模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   监控层   → 由外部调用Battery_GetVoltageAnal()比较阈值，控制LED
 *   调度层   → Battery_GetFlag()驱动主循环任务调度
 *   算法层   → Battery_GetVoltage()完成ADC值→电压的线性换算
 *   反馈处理层→ 分压比校正（R6/R14），VREF+基准确认
 *   驱动层   ← 本模块，激活TIM2/ADC1硬件，产生JEOC中断
 *
 * 对外接口（.h文件声明，外部只能通过这些函数访问）：
 *   Battery_GetVoltageAnal() → 获取原始ADC值（用于阈值判断）
 *   Battery_GetVoltage()     → 获取换算后电压（用于调试打印）
 *   Battery_GetFlag()        → 查询是否有新数据（消费后自动清零）
 *   ADC_Battery_TIM_Init()   → TIM2初始化
 *   ADC_Battery_Init()       → ADC1初始化
 *   NVIC_Battery_Init()      → 中断控制器初始化
 *
 * 【防御性编程原则】
 *   1. static  → 内部变量不对外暴露，外部只能通过接口函数访问
 *   2. volatile → 中断和主循环共享的变量必须加volatile，
 *                 防止编译器优化导致主循环读到缓存旧值
 *   3. flag机制 → 中断只举旗，主循环负责处理，职责分离
 *   4. GetFlag()读完自动清零 → 防止同一次数据被重复处理
 * ================================================================
 */

/* ----------------------------------------------------------------
 * 模块内部状态（static封装，外部不可直接访问）
 *
 * 【为什么加volatile？】
 *   Voltage_Anal和JEOC_Status在中断里被修改，在主循环里被读取。
 *   编译器不知道中断的存在，可能把变量缓存在寄存器里，
 *   导致主循环永远读到旧值。volatile强制每次从内存读取最新值。
 *
 * 【为什么加static？】
 *   数据封装在本文件内，外部文件看不到，只能通过接口函数访问。
 *   .h文件里只放函数声明，不放变量——变量永远在.c文件里定义。
 * ---------------------------------------------------------------- */
static volatile uint16_t Voltage_Anal;   // ADC原始值，0~4095
static volatile uint8_t  JEOC_Status;    // 新数据标志位，1=有新数据


static volatile float Vbat = 0.0f;
/* ----------------------------------------------------------------
 * [反馈处理层接口] 获取原始ADC值
 *
 * 用途：监控层阈值判断，直接比较ADC值效率更高，避免浮点运算
 * 阈值对照（由硬件参数推导，分压比=3.3/8.4）：
 *   > 3399 → 电量70%~100%（三灯亮）
 *   > 3193 → 电量40%~70% （两灯亮）
 *   > 2990 → 电量10%~40% （一灯亮）
 *   ≤ 2990 → 电量<10%    （全灭，需充电）
 * ---------------------------------------------------------------- */
uint16_t Battery_GetVoltageAnal(void)
{
    return Voltage_Anal;
}

/* ----------------------------------------------------------------
 * [反馈处理层接口] 获取换算后的实际电压值
 *
 * 换算公式推导：
 *   V_bat = ADC值 × (VREF+ / 4095) × (1 / 分压比)
 *         = ADC值 × (3.3 / 4095) × (8.4 / 3.3)
 *         = ADC值 × (8.4 / 4095)    ← 3.3约分消掉
 *
 * 【为什么加f后缀？】
 *   3.3和4095默认是double类型，STM32上double运算慢。
 *   加f后缀明确为float，运算更快。
 *
 * 用途：调试打印，人可读的电压值（配合sprintf使用）
 * ---------------------------------------------------------------- */
// float Battery_GetVoltage(void)
// {
//     float Voltage = Voltage_Anal * (3.3f / 4095.0f) * (8.4f / 3.3f);
//     return Voltage;
// }

float Battery_GetVoltage(void)
{
    return Vbat + 1.4f;
}

/* ----------------------------------------------------------------
 * [调度层接口] 查询是否有新的ADC数据
 *
 * 读取后自动清零——防止同一次数据被主循环重复处理。
 * 这是"消费型flag"的标准写法：
 *   中断举旗(JEOC_Status=1) → 主循环消费(读走并清零) → 等待下次中断
 *
 * 【前后台模型的体现】
 *   中断（前台）：只负责采数据、举旗，越快越好
 *   主循环（后台）：看到旗子才执行业务逻辑（比较阈值、控制LED）
 *   中断不直接控制LED，避免中断里执行过多逻辑影响实时性
 * ---------------------------------------------------------------- */
uint8_t Battery_GetFlag(void)
{
    uint8_t temp = JEOC_Status;
    JEOC_Status = 0;          // 读完清零，防止重复消费
    return temp;
}

/**
 * ================================================================
 * [驱动层] ADC电池电压采集 - 定时器初始化
 * ================================================================
 *
 * 职责：
 *   配置TIM2为10ms周期定时器，通过TRGO硬件触发ADC注入组采样。
 *   本函数是ADC定时触发的唯一硬件入口，上层无需关心定时细节。
 *
 * 硬件约束（来自原理图）：
 *   TIM2挂载于APB1总线，系统时钟72MHz
 *
 * 定时参数推导：
 *   预分频 = 72-1  → 分频后计数时钟 = 1MHz = 每计数1微秒
 *   ARR    = 10000-1 → 计数10000次溢出 = 10ms周期
 *   触发频率 = 100Hz（每秒触发100次ADC采样）
 *
 * 【曾犯的错误】
 *   ARR写成100-1：100次计数=100微秒=0.1ms，不是10ms
 *   根本原因：ARR的单位是"计数次数"，不是频率
 *   推导顺序：目标周期(10ms) → 换算成微秒(10000μs) → 填入ARR
 *
 * 【曾有的疑问】
 *   Q: 为什么用TRGO而不是TIM2_CH1触发ADC？
 *   A: TRGO是TIM内部信号，走芯片内部总线直连ADC触发输入，不经过任何引脚。
 *      CH1是引脚信号，需要出芯片再回来，多此一举。
 *      TRGO = TIM对外喊"我到时间了"的内部广播
 *
 *   Q: TIM_TRGOSource_Enable不是使能TRGO吗？
 *   A: 这个参数不是"要不要使能"，而是"TRGO在什么时机输出信号"
 *      Enable = TIM启动瞬间输出一次（只触发一次ADC，不是我们要的）
 *      Update = 每次计数器溢出时输出（周期性触发，这才是我们要的）
 *
 *   Q: TIM有主模式和从模式，TRGO属于哪个？
 *   A: TRGO属于主模式输出——TIM2是主，ADC是被触发的从
 *      从模式是TIM听别人指挥（如定时器级联）
 * ================================================================
 */
void ADC_Battery_TIM_Init(void)
{
    /* [1] 开启TIM2时钟
     * TIM2挂载APB1总线（低速，最高36MHz，经倍频后72MHz提供给TIM） */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* [2] 配置时基参数 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 72 - 1;               // 72MHz→1MHz，每计数1μs
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         // 时钟分割，保持默认
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;   // 向上计数，0→ARR溢出
    TIM_TimeBaseInitStructure.TIM_Period        = 10000 - 1;            // 10000μs = 10ms溢出

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* [3] 配置TRGO输出源为Update事件
     * 每次计数器溢出（Update事件）时，TRGO向ADC发出触发信号
     * 注意：TIM_TRGOSource_Enable ≠ 使能TRGO，而是"启动时触发一次" */
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
 * 【曾犯的错误】
 *   1. ContinuousConvMode = ENABLE
 *      错误原因：已有TIM定时触发，不需要ADC自己连续转换
 *      连续模式会让ADC停不下来，浪费功耗，与定时触发冲突
 *      正确：DISABLE，触发一次转换一次
 *
 *   2. ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO
 *      两个错误：①用的是TIM2不是TIM3
 *               ②ADC_InitTypeDef里的ExternalTrigConv是规则组参数
 *      注入组触发源必须用专属函数ADC_ExternalTrigInjectedConvConfig单独配置
 *
 *   3. 忘记调用ADC_Init()
 *      填完结构体必须调用Init才能写入寄存器生效
 *
 *   4. 校准等待写成 while(ADC_GetCalibrationStatus == RESET)
 *      错误①：少了括号和参数，应为ADC_GetCalibrationStatus(ADC1)
 *      错误②：条件反了，SET=校准进行中，RESET=校准完成
 *      正确：while(ADC_GetCalibrationStatus(ADC1) == SET)
 *      口诀：等到"好了"才动手，SET=还没好，RESET=好了
 *
 *   5. ADC_ExternalTrigInjectedConvCmd调用了两次
 *      第一次在ADC_Cmd之前，ADC未使能时配置无效
 *      正确位置：ADC使能→校准完成→再使能外部触发
 *
 *   6. 模拟输入引脚配置了GPIO_Speed
 *      GPIO_Mode_AIN模式下Speed参数无意义，不需要配置
 *
 * 【曾有的疑问】
 *   Q: 为什么用注入组而不是规则组？
 *   A: 硬件约束决定——TIM2_TRGO只出现在注入组的触发源列表里
 *      规则组触发源列表里没有TIM2_TRGO，所以被迫选注入组
 *      注入组额外优点：专属JDR1寄存器、专属JEOC中断、可打断规则组
 *
 *   Q: ADC_InitTypeDef结构体配置的是什么？
 *   A: 只配置规则组，注入组需要单独调用专属函数：
 *      ADC_InjectedChannelConfig         → 注入组通道和采样时间
 *      ADC_ExternalTrigInjectedConvConfig → 注入组外部触发源
 *      ADC_ExternalTrigInjectedConvCmd    → 使能注入组外部触发
 *
 *   Q: 连续模式、扫描模式、自动注入各是什么？
 *   A: 连续模式 → 转换完立刻再转，我们用TIM触发所以关闭
 *      扫描模式 → 按顺序转换多个通道，我们只有一个通道所以关闭
 *      自动注入 → 规则组完成后自动触发注入组，我们直接触发注入组所以关闭
 *
 *   Q: VREF+怎么确认是3.3V？
 *   A: 查引脚图，VREF+接的是3V3电源轨，由MT2492升压模块提供
 *      VREF+决定ADC满量程，设计者故意让8.4V满电→分压3.3V→ADC 4095
 * ================================================================
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
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;              // 单次转换，由TIM触发，不自动连续
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;  // 右对齐，结果在低12位，直接读0~4095
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None; // 规则组不用外部触发
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent; // 独立模式，ADC1独立工作
    ADC_InitStructure.ADC_NbrOfChannel       = 1;                    // 规则组转换1个通道
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;              // 单通道不需要扫描模式
    ADC_Init(ADC1, &ADC_InitStructure);                              // 必须调用Init才能生效！

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
 * 【曾有的疑问】
 *   Q: 抢占优先级和子优先级有什么区别？
 *   A: 抢占优先级 → 决定能不能插队打断正在执行的中断（数字越小越高）
 *      子优先级   → 同等抢占时决定排队顺序，不能互相打断
 *
 *   Q: NVIC_PriorityGroup_2和具体优先级数字的关系？
 *   A: Group_2规定了4位优先级的切分方式（2位抢占+2位子优先级）
 *      具体中断再在这个范围内（0~3）填自己的优先级数字
 *      先定规则，再按规则填数字
 * ================================================================
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




/**
 * ================================================================
 * [驱动层] ADC注入组转换完成中断处理函数
 * ================================================================
 *
 * 触发条件：ADC1注入组转换完成（JEOC标志位置位）
 *
 * 【前后台模型的体现】
 *   中断（前台）：只做最紧迫的事——读数据、举旗，立刻退出
 *   主循环（后台）：看到旗子才执行业务逻辑（阈值判断、控制LED）
 *
 * 【为什么不在中断里直接控制LED？】
 *   中断应该越短越好，业务逻辑放主循环。
 *   如果在中断里做太多事，会影响其他中断的响应时间。
 *
 * 【防御性编程】
 *   用if判断JEOC标志位，防止误触发（虽然ADC1_2_IRQHandler
 *   只响应ADC1和ADC2，加判断是好习惯）
 * ================================================================
 */
// void ADC1_2_IRQHandler(void)
// {
//     if(ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == SET)
//     {
//         /* 读取注入组第1位结果寄存器JDR1，存入模块内部变量 */
//         Voltage_Anal = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);

//         /* 举旗通知主循环：有新数据了 */
//         JEOC_Status = 1;

//         /* 清除中断标志位，否则会反复触发中断 */
//         ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
//     }
// }


