#ifndef	__GPIO_LED_H
#define	__GPIO_LED_H
#include "stm32f10x.h"


void LED_GPIO_Init(void);

void LED_GPIO_SetBits(uint8_t led1, uint8_t led2, uint8_t led3);



#endif	//!__GPIO_LED_H


