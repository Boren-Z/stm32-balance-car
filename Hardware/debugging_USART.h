#ifndef	__DEBUGGING_USART
#define __DEBUGGING_USART
#include "stm32f10x.h"

/*
 * [Five-Layer] Debug Infrastructure - USART Debug Module
 *
 * This module sits outside the five-layer stack. It's an observation 
 * window used by every layer to output data, not business logic itself 
 * (think: scaffolding, not part of the building, but you can't build without it).
 *
 * Monitor layer   : prints alarms, low-voltage warnings
 * Scheduler layer : sends telemetry every 100ms
 * Algorithm layer : prints PID intermediate values for tuning
 * Feedback layer  : prints raw IMU data, filtered angle
 * Driver layer    : this module, configures USART2 hardware
 *
 * Debug channel comes first in the dev sequence:
 * USART debug channel -> sensors -> control algorithm
 */

//Initialization
void Debugging_USART_Init(void);

// Send data (single byte)
void Debugging_USART_SendByte(uint8_t info);

// Send data (multiple bytes)
void Debugging_USART_SendBytes(uint8_t *array, uint16_t size);

// Send string
void Debugging_USART_SendString(const char *string);

// Receive data
uint8_t Debugging_USART_Receivebyte(void);

// Receive multiple bytes
void Debugging_USART_Receivebytes(uint8_t *array, uint16_t size);

// Print an error message via USART (used internally on invalid args)
static void report_error(const char *msg);

#endif //!__DEBUGGING_USART
