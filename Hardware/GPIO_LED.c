#include "stm32f10x.h"

/**
 * ================================================================
 * 【五层塔定位】监控层执行器 - 电量指示LED模块
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   监控层  → 调用LED_GPIO_SetBits()输出电量状态（直接使用者）
 *   驱动层  ← 本模块，配置GPIO硬件，提供SetBits接口
 *
 * 硬件约束（来自原理图）：
 *   LED1 → PA4（电量100%~70%指示）
 *   LED2 → PA5（电量70%~40%指示）
 *   LED3 → PA6（电量40%~10%指示）
 *   全灭  → 电量<10%，提示充电
 *
 * 电量档位与LED状态对应（ADC原始值阈值由硬件参数推导）：
 *   ADC > 3399 → LED_GPIO_SetBits(1,1,1) 三灯亮（100%~70%）
 *   ADC > 3193 → LED_GPIO_SetBits(0,1,1) 两灯亮（70%~40%）
 *   ADC > 2990 → LED_GPIO_SetBits(0,0,1) 一灯亮（40%~10%）
 *   ADC ≤ 2990 → LED_GPIO_SetBits(0,0,0) 全灭（<10%）
 *
 * 对外接口：
 *   LED_GPIO_Init()    → 初始化GPIO硬件（在main.c最先调用）
 *   LED_GPIO_SetBits() → 控制三个LED的亮灭状态
 *
 * 【设计决策】
 *   最初设计了六个函数（SetPin4/ResetPin4/SetPin5...），
 *   重构为一个LED_GPIO_SetBits(led1, led2, led3)。
 *   原因：三个LED状态总是一起设置，六个函数高度重复，
 *   一个函数参数化更清晰，调用方一眼看出亮灭状态。
 * ================================================================
 */

/**
 * ================================================================
 * [驱动层] LED GPIO初始化
 * ================================================================
 *
 * 职责：配置PA4/PA5/PA6为推挽输出，为上层提供LED控制能力。
 *
 * 硬件约束：
 *   PA4/PA5/PA6均属于GPIOA，挂载APB2总线
 *   推挽输出：能主动驱动高低电平，适合直接驱动LED
 *
 * 【和串口GPIO配置的对比】
 *   串口TX → 复用推挽输出（AF_PP）：引脚由USART外设控制
 *   LED    → 普通推挽输出（Out_PP）：引脚由GPIO寄存器直接控制
 *   区别在于"谁来驱动这个引脚"
 *
 * 【三个引脚可以合并到一次GPIO_Init】
 *   PA4|PA5|PA6模式相同，可以用|运算符合并，一次Init搞定。
 *   串口TX和RX模式不同（一个输出一个输入），所以需要两次Init。
 * ================================================================
 */
void LED_GPIO_Init(void)
{
    /* 开启GPIOA时钟，PA4/PA5/PA6均属于GPIOA，挂载APB2总线 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    /* 三个引脚模式相同，用|合并，一次Init即可
     * 推挽输出：主动驱动高低电平，适合控制LED */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * ================================================================
 * [监控层接口] 设置三个LED的亮灭状态
 * ================================================================
 *
 * 参数：
 *   led1 → PA4，非零=亮，零=灭
 *   led2 → PA5，非零=亮，零=灭
 *   led3 → PA6，非零=亮，零=灭
 *
 * 调用示例：
 *   LED_GPIO_SetBits(1,1,1);  // 三灯全亮
 *   LED_GPIO_SetBits(0,1,1);  // 两灯亮
 *   LED_GPIO_SetBits(0,0,1);  // 一灯亮
 *   LED_GPIO_SetBits(0,0,0);  // 全灭
 *
 * 【三元运算符的使用】
 *   led1 ? Bit_SET : Bit_RESET
 *   含义：led1非零 → Bit_SET（亮），led1为零 → Bit_RESET（灭）
 *   适用于任何非此即彼的场景，等价于if/else但更紧凑
 *   嵌入式里常见用法：
 *     方向控制：forward ? Bit_SET : Bit_RESET
 *     状态指示：error ? Bit_SET : Bit_RESET
 *
 * 【为什么不设计成六个函数？】
 *   最初设计：SetPin4/ResetPin4/SetPin5/ResetPin5/SetPin6/ResetPin6
 *   问题：调用方需要连续调用三次，而且看不出整体状态
 *   重构后：一次调用传三个参数，调用方一眼看出亮灭组合
 *   原则：接口设计跟着使用场景走，不跟着硬件结构走
 * ================================================================
 */
void LED_GPIO_SetBits(uint8_t led1, uint8_t led2, uint8_t led3)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, led1 ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, led2 ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_6, led3 ? Bit_SET : Bit_RESET);
}
