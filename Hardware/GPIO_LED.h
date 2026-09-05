#ifndef	__GPIO_LED_H
#define	__GPIO_LED_H
#include "stm32f10x.h"

/**
 * [Five-Layer Placement] Monitoring-layer actuator - Battery-level indicator LED module
 * 
 * This module's position in the five-layer :
 * Monitoring layer -> Calls LED_GPIO_SetBits() to output battery-level status (direct consumer)
 * Driver layer     <- This module, configures GPIO hardware, provides the SetBits interface
 * 
 * Hardware:
 * LED1 → PA4 (indicates battery 100%~70%)
 * LED2 → PA5 (70%~40%)
 * LED3 → PA6 (40%~10%)
 * All off → battery <10%, prompts charging
 * 
 * Battery level tiers mapped to LED states (raw ADC thresholds derived from hardware parameters):
 * ADC > 3399 → LED_GPIO_SetBits(1,1,1) three LEDs on 100%~70%
 * ADC > 3193 → LED_GPIO_SetBits(0,1,1) two LEDs on 70%~40%
 * ADC > 2990 → LED_GPIO_SetBits(0,0,1) one LED on 40%~10%
 * ADC ≤ 2990 → LED_GPIO_SetBits(0,0,0) all off <10%
*/

void LED_GPIO_Init(void);

void LED_GPIO_SetBits(uint8_t led1, uint8_t led2, uint8_t led3);


#endif	//!__GPIO_LED_H


