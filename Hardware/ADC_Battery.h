#ifndef	__ADC_BATTERY_H
#define	__ADC_BATTERY_H
#include "stm32f10x.h"

/**
 *  [Five-Layer Placement] Driver layer - ADC battery voltage acquisition module
 *
 *   This module's role across the five layers:
 *
 *   Monitoring layer    -> Externally calls Battery_GetVoltageAnal() to compare against thresholds and control LEDs
 *   Scheduling layer    -> Battery_GetFlag() drives main-loop task scheduling
 *   Algorithm layer     -> Battery_GetVoltage() performs the linear ADC-value-to-voltage conversion
 *   Feedback processing -> Divider-ratio correction (R6/R14), VREF+ reference confirmation
 *   Driver layer        <- This module, activates TIM2/ADC1 hardware, generates the JEOC interrupt
 *
 */

uint16_t Battery_GetVoltageAnal(void);

float Battery_GetVoltage(void);

uint8_t Battery_GetFlag(void);

void ADC_Battery_TIM_Init(void);

void ADC_Battery_Init(void);

void NVIC_Battery_Init(void);

#endif //!__ADC_BATTERY_H

