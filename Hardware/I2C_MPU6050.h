#ifndef	__I2C_MPU6050_H
#define	__I2C_MPU6050_H

#include "stm32f10x.h"

/**
 * [Five-Layer Placement] Driver layer - I2C/MPU6050 attitude sensor driver module
 *
 * This module's position in the five layers:
 *
 *   Feedback processing layer -> Calls MPU6050_GetData() to get raw six-axis
 *                                 data; the complementary filter algorithm at
 *                                 this layer converts the raw data into pitch angle
 *   Driver layer              <- This module, activates the I2C1 hardware
 *                                 peripheral + MPU6050 register configuration
 *
 * Hardware architecture:
 *   STM32 I2C1 (remapped) <-> MPU6050 (attitude sensor, I2C slave)
 *
 *   I2C1_SCL -> PB8 (after remapping; PB6 is occupied by the right motor by default)
 *   I2C1_SDA -> PB9 (after remapping; PB7 is occupied by the right motor by default)
 *
 *   MPU6050 hardware state:
 *     AD0   -> GND  -> I2C slave address = 0x68 (7-bit) -> 0xD0 write / 0xD1 read (8-bit bus byte)
 *     nCS   -> 3V3  -> Forces I2C mode (pulling it low switches to SPI mode, breaking I2C entirely)
 *
 *   The protocol layer (I2C_GPIO_Init/I2C1_Protocol_Init) and the device
 *   layer (MPU6050_Init) are strictly separated:
 *   the former only cares about "how STM32 gets I2C running," and can be
 *   reused directly for any I2C device;
 *   the latter only cares about "what values this specific MPU6050 chip
 *   needs to be written with"
 *   This follows the same principle as separating GPIO initialization from
 *   PWM parameter initialization in Double_Motors_Init()
 *
 */



/* 
 * Raw six-axis data, in LSB units (not yet converted to physical units 
 * that conversion happens in the feedback processing layer) 
 */
typedef struct
{
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;

    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
} MPU6050_Info;

/* Initialize the I2C bus + MPU6050 registers, only needs to be called once */
void I2C_MPU6050_Init(void);

/* Read the WHO_AM_I register, used to verify I2C communication was established correctly */
uint8_t MPU6050_GetID(void);

/* Read raw six-axis data, results are written into the struct pointed to by Data */
void MPU6050_GetData(MPU6050_Info* Data);

#endif	//!__I2C_MPU6050_H

