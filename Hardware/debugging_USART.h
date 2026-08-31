#ifndef	__DEBUGGING_USART
#define __DEBUGGING_USART
#include "stm32f10x.h"
//初始化函数
void Debugging_USART_Init(void);

//发送数据函数(单个)
void Debugging_USART_SendByte(uint8_t info);
//发送数据函数(多个)
void Debugging_USART_SendBytes(uint8_t *array, uint16_t size);
//发送字符串
void Debugging_USART_SendString(const char *string);

//接收数据
uint8_t Debugging_USART_Receivebyte(void);

void Debugging_USART_Receivebytes(uint8_t *array, uint16_t size);

static void report_error(const char *msg);



#endif //!__DEBUGGING_USART
