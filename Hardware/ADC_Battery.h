#ifndef	__ADC_BATTERY_H
#define	__ADC_BATTERY_H
#include "stm32f10x.h"


uint16_t Battery_GetVoltageAnal(void);

float Battery_GetVoltage(void);

uint8_t Battery_GetFlag(void);

void ADC_Battery_TIM_Init(void);

void ADC_Battery_Init(void);

void NVIC_Battery_Init(void);

#endif //!__ADC_BATTERY_H

