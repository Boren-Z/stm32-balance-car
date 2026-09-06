#ifndef __SYSTEM_TIM_H
#define __SYSTEM_TIM_H
#include "stm32f10x.h"

/**
 * [Five-Layer Placement] Scheduling layer core - System time base module
 *
 * This module's position in the five layers:
 *
 *   Monitoring layer      → Uses System_GetTick() to implement timeout protection (e.g. sustained low-voltage detection)
 *   Scheduling layer      ← This module's core responsibility, provides the global time reference
 *   Algorithm layer       → PID computation depends on a fixed time interval Δt
 *   Feedback processing   → Controls sensor sampling periods
 *   Driver layer          → SysTick hardware configuration
 *
 * Core idea:
 *   SysTick generates an interrupt every 1ms, accumulating System_Tick.
 *   Every task decides whether it should run via "current tick - last-run
 *   tick >= target interval".
 *   SysTick is the ruler; each task measures its own time, the CPU never
 *   needs to wait around dedicated to any one task.
 *
 * Task scheduling model (foreground/background):
 *   SysTick interrupt (foreground) → only does tick++, as fast as possible
 *   Main loop (background)         → uses tick to decide whether each task should run
 *
 *   static uint32_t last_xxx = 0;
 *   if(System_GetTick() - last_xxx >= target_interval_ms)
 *   {
 *       last_xxx = System_GetTick();
 *       // run the task
 *   }
 *
 * Typical task schedule:
 *   Every 1ms   → inner-loop PID (most urgent, run directly inside the interrupt)
 *   Every 5ms   → outer-loop PID
 *   Every 10ms  → ADC battery voltage check
 *   Every 100ms → send debug data over the serial port
 *
 * Analogy to CODESYS:
 *   CODESYS Task configuration ↔ System_Tick + main loop
 *   CODESYS manages the scan cycle automatically; bare metal has to
 *   implement it itself with SysTick
 *   A PLC has its own scan engine — on STM32, you are that scan engine
 *
 * Difference from FreeRTOS:
 *   FreeRTOS: preemptive, a higher-priority task can interrupt a lower-priority one
 *   This module: cooperative, the main loop scans sequentially, each task decides for itself whether to run
 *   SysTick is the clock, not the scheduler; the main loop is the scheduler
 *
 */

/* ============================================================
 * New scheme: SysTick microsecond-level time base (aligned with reference code)
 * ============================================================ */
void     System_Init(void);
uint32_t System_GetTick(void);
uint64_t System_GetUs(void);

#endif  //!__SYSTEM_TIM_H

