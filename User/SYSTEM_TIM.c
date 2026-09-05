#include "stm32f10x.h"

/**
 * ================================================================
 * 【五层塔定位】调度层核心 - 系统时基模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   监控层   → 用System_GetTick()实现超时保护（如低压持续检测）
 *   调度层   ← 本模块核心职责，提供全局时间基准
 *   算法层   → PID计算依赖固定时间间隔Δt
 *   反馈处理层→ 传感器采样周期控制
 *   驱动层   → TIM3硬件配置
 *
 * 核心思想：
 *   TIM3每1ms产生一次中断，System_Tick累加。
 *   所有任务通过"当前tick - 上次执行tick >= 目标间隔"判断是否该执行。
 *   TIM3是尺子，任务自己量时间，CPU不需要专门等待。
 *
 * 任务调度模式（前后台模型）：
 *   TIM3中断（前台）→ 只做tick++，越快越好
 *   主循环（后台）  → 用tick判断各任务是否该执行
 *
 *   static uint32_t last_xxx = 0;
 *   if(System_GetTick() - last_xxx >= 目标间隔ms)
 *   {
 *       last_xxx = System_GetTick();
 *       // 执行任务
 *   }
 *
 * 典型任务调度表：
 *   每1ms   → 内环PID（最紧迫，直接在中断里执行）
 *   每5ms   → 外环PID
 *   每10ms  → ADC电池电压检测
 *   每100ms → 串口打印调试数据
 *
 * 与CODESYS的类比：
 *   CODESYS Task配置 ↔ System_Tick + 主循环
 *   CODESYS自动管理扫描周期，裸机需要自己用TIM3实现
 *   PLC有自己的扫描引擎，STM32你就是那个扫描引擎
 *
 * 与FreeRTOS的区别：
 *   FreeRTOS：抢占式，高优先级任务可以打断低优先级任务
 *   本模块：协作式，主循环顺序扫描，任务自己决定是否执行
 *   TIM3是时钟，不是调度器；主循环才是调度器
 *
 * 对外接口：
 *   TIM_System_Schedule() → TIM3初始化
 *   NVIC_System_Init()    → 中断控制器初始化
 *   System_GetTick()      → 获取当前tick值（毫秒级时间戳）
 *
 * 【曾有的疑问】
 *   Q: 为什么不直接用系统时钟SysTick？
 *   A: SysTick是Cortex-M3内核自带定时器，HAL库默认用它。
 *      标准库裸机开发SysTick未被自动配置，用TIM3是标准做法。
 *      理解了TIM3的配置，再看HAL库的SysTick会一眼看透。
 *
 *   Q: uint32_t会不会溢出？
 *   A: 会，49.7天后归零。但不影响时间差计算：
 *      无符号整数溢出后差值运算依然正确（利用补码特性）
 *      只要两次时刻间隔不超过49天，差值永远准确
 * ================================================================
 */

/* ----------------------------------------------------------------
 * 系统时间戳（模块内部，通过System_GetTick()对外暴露）
 *
 * 【为什么加volatile？】
 *   System_Tick在TIM3中断里被修改，在主循环里被读取。
 *   volatile禁止编译器优化，强制每次从内存读取最新值。
 *   涉及中断的共享变量，volatile跑不了。
 *
 * 【为什么加static？】
 *   数据封装在本文件内，外部通过System_GetTick()接口访问，
 *   不直接暴露内部变量。
 * ---------------------------------------------------------------- */


/* ============================================================
 * 新方案：SysTick微秒级时基
 * SysTick是ARM Cortex-M3内核自带定时器，不占用任何片上外设
 * TIM1/2/3/4全部释放给业务使用
 * ============================================================ */

/* VAR：内部状态 */
static volatile uint64_t System_Tick_Ms = 0;   // 毫秒计数
static float us_per_mini_tick = 0.0f;          // 每个SysTick计数对应多少微秒


/**
 * ================================================================
 * [调度层接口] 获取当前系统时间戳
 * [调度层] 获取毫秒时间戳（接口不变，兼容所有调用方）
 * ================================================================
 *
 * 返回值：自系统启动以来的毫秒数（1ms精度）
 *
 * 使用模式：
 *   static uint32_t last = 0;
 *   if(System_GetTick() - last >= 100)  // 每100ms执行一次
 *   {
 *       last = System_GetTick();
 *       // 执行任务
 *   }
 *
 * 注意：调用两次System_GetTick()之间tick可能已经改变，
 * 如需确保一致性，先存到局部变量再使用：
 *   uint32_t now = System_GetTick();
 * ================================================================
 */
uint32_t System_GetTick(void)
{
//     return System_Tick;
    return (uint32_t)System_Tick_Ms;
}






/**
 * ================================================================
 * [调度层] SysTick初始化（替代TIM_System_Schedule+NVIC_System_Init）
 * ================================================================
 */
void System_Init(void)
{
    RCC_ClocksTypeDef clockinfo = {0};
    RCC_GetClocksFreq(&clockinfo);

    uint32_t load = clockinfo.HCLK_Frequency / 1000 - 1;  // 1ms周期
    SysTick->LOAD = load;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   // AHB时钟源
                  | SysTick_CTRL_TICKINT_Msk      // 使能中断
                  | SysTick_CTRL_ENABLE_Msk;      // 使能计数

    SCB->SHP[7] = 0;  // SysTick优先级最高

    us_per_mini_tick = 1000.0f / (load + 1);
}

/**
 * ================================================================
 * [调度层] SysTick中断处理函数
 * ================================================================
 */
void System_Tick_Update(void)  
{
    System_Tick_Ms++;
}



/**
 * ================================================================
 * [调度层] 获取微秒时间戳（新增）
 * ================================================================
 *
 * 原理：毫秒部分×1000 + SysTick计数器余量折算成微秒
 * 原子保护：读取过程中关闭中断，防止tick和VAL不一致
 * ================================================================
 */
uint64_t System_GetUs(void)
{
    uint64_t tick;
    uint32_t mini_tick;

    __disable_irq();
    while(1)
    {
        tick = System_Tick_Ms;
        mini_tick = SysTick->VAL;
        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG)
            System_Tick_Ms++;
        else
            break;
    }
    __enable_irq();

    tick *= 1000;
    tick += (uint32_t)((SysTick->LOAD - mini_tick) * us_per_mini_tick);
    return tick;
}


