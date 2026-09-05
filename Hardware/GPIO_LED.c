#include "stm32f10x.h"

/*[Driver layer] LED GPIO initialization*/
void LED_GPIO_Init(void)
{
    /* Enable GPIOA clock — PA4/PA5/PA6 all belong to GPIOA, mounted on the APB2 bus */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * [Monitoring layer interface] Set the on/off state of the three LEDs
 *
 * Parameters:
 * led1 → PA4, nonzero = on, zero = off
 * led2 → PA5, nonzero = on, zero = off
 * led3 → PA6, nonzero = on, zero = off
 */
void LED_GPIO_SetBits(uint8_t led1, uint8_t led2, uint8_t led3)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, led1 ? Bit_SET : Bit_RESET); // Ternary operator
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, led2 ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_6, led3 ? Bit_SET : Bit_RESET);
}
