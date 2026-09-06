#include "stm32f10x.h"


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


