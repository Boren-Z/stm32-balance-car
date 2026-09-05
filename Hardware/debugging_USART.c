#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

static void report_error(const char *msg);

/*
 * [Driver Layer] Debug USART initialization
 * Responsibility: Bring up USART2 hardware, providing the debug send/receive channel for upper layers.
 */

void Debugging_USART_Init(void)
{
    /* [1] Turn on peripheral clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);      //USART2 is on the APB1 bus, System clock: 72MHz
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    /* [2] PA2 is configured as a multiplexed push-pull output (TX) */
    // Multiplexing: The pin is controlled by the USART2 peripheral, not directly controlled by the GPIO register
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;            
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                 // PA2 is for USART2_TX (transmit)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                      // TX Init alone once

    /* [3] PA3 is configured as a pull-up input (RX) */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;                 // PA3 is for USART2_RX (receive)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                      // RX Init alone once

    /* [4] Configure USART2 communication parameters (115200 8N1) */
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 115200;                         //standard debug baud, supported by nearly all tools
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;                //no parity overhead needed for debugging
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;               //tells the receiver the frame ended, line idles high
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;            //standard byte transfer
    USART_Init(USART2, &USART_InitStructure);

    /* [5] Power on USART2, the hardware begins to work. */
    USART_Cmd(USART2, ENABLE);
}

/* 
 * [Driver layer] Send one byte 
 * 
 * Flag bit: 
 * TXE = Transmit data register empty 
 * TC = Transmission completed 
 * RXNE = Receive data register is not empty 
 */
void Debugging_USART_SendByte(uint8_t info)
{
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, info);
}

/* [Driver Layer] Send multiple bytes  */
void Debugging_USART_SendBytes(uint8_t *array, uint16_t size)
{

/* [Defensive Programming] Check the two most dangerous parameters: 
 * array == NULL -> null pointer, access will crash 
 * size == 0     -> There is no data to send, the loop is meaningless 
 */
    if (array == NULL || size == 0)
    {// Report errors through report_error instead of silently 
        report_error("[ERROR] SendBytes: invalid args\n");
        return;
    }

    for (int i = 0; i < size; i++)
    {
        Debugging_USART_SendByte(array[i]);
    }
}

/* [Driver layer] Send string */
// The essence of string: char array + '\0' terminator,
// strlen calculates the number of characters (excluding '\0'), SendBytes sends bytes
void Debugging_USART_SendString(const char *string)
{
    if (string == NULL || strlen(string) == 0)
    {   /*[Defensive Programming]: Check NULL and empty strings to prevent invalid calls*/
        report_error("[ERROR] SendString: invalid args\n");
        return;
    }
    // Requires (uint8_t *) cast, the bottom layer of the two is exactly the same, but the semantic labels are different
    Debugging_USART_SendBytes((uint8_t *)string, strlen(string));
}

/* [Driver layer] Receive a byte (lowest layer receiving function) */
uint8_t Debugging_USART_Receivebyte(void)
{   // Wait for RXNE (the receiving register is not empty) -> read the data register and wait until "OK"
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);  

/*  
    This is a blocking reception and will wait until there is data. 
    If the other party does not send data, the program will freeze.
*/
    return (uint8_t)USART_ReceiveData(USART2);                      
}

/* [Driver layer] Receive multiple bytes */
void Debugging_USART_Receivebytes(uint8_t *array, uint16_t size)
{
    /* [Defensive Programming]: Check NULL and empty strings to prevent invalid calls */
    if (array == NULL || size == 0)
    {
        report_error("[ERROR] ReceiveBytes: invalid args\n");
        return;
    }

    for (int i = 0; i < size; i++)
    {
        array[i] = Debugging_USART_Receivebyte();
    }
/* 
    [To be improved] The Timeout parameter is missing: 
    the current version will freeze if the other party does not send data. 
    Can rely on System_GetTick() to add a timeout.
*/
}

/* [Internal] Error reporting function */
static void report_error(const char *msg)
{
    while (*msg != '\0')                 // Stop when encountering the end of string character
    {
        Debugging_USART_SendByte(*msg);  // Only rely on the lowest level function to avoid recursion
        msg++;                           // Next character
    }
}