#include "stm32f10x.h"
#include "MPU6050_Reg.h"
#include "I2C_MPU6050.h"

// MPU6050's I2C slave address (7-bit 0x68 shifted left by 1, LSB reserved for R/W)
#define MPU6050_ADDRESS		0xD0		

/* [Driver layer] I2C1 pin initialization (static, unrelated to MPU6050 — protocol layer) */
static void I2C_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    // AFIO clock must be enabled before remapping
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO  , ENABLE);    
    // PB6/PB7 are occupied by the right motor by default, so remapping is required
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);             
    // Alternate-function open-drain, required by the I2C hardware

    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;           
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* [Driver layer] I2C1 bus protocol parameter initialization (unrelated to MPU6050 — protocol layer); 
 * configures I2C1's communication speed, mode, and acknowledgment rules 
**/


static void I2C1_Protocol_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_ClockSpeed            = 400000;         // Fast mode 400kHz, prioritizing stability during debugging
    I2C_InitStructure.I2C_Mode                  = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle             = I2C_DutyCycle_2;// Legal in Fast mode too (16_9 more common); harmless here
    I2C_InitStructure.I2C_OwnAddress1           = 0x00;           // STM32 acts as master, this value is never actually used
    I2C_InitStructure.I2C_Ack                   = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress   = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

/* [Driver layer] I2C event wait */
static void MPU6050_WaitEvent(uint32_t I2C_EVENT)
{
    uint32_t Timeout = 10000;
    while(I2C_CheckEvent(I2C1, I2C_EVENT) != SUCCESS)
    {
        Timeout--;
        if(Timeout == 0) break;  
    }
}

/**
 * [Driver layer] Write MPU6050 register
 *
 * I2C write timing: START → Address(Write) → EV6 → reg byte → EV8 → data byte → EV8_2 → STOP
 */

static void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);               // EV5

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); // EV6

    I2C_SendData(I2C1, reg);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);         // EV8

    I2C_SendData(I2C1, data);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);          // EV8_2，Confirmed that it has really been sent.

    I2C_GenerateSTOP(I2C1, ENABLE);
}
/* [Driver layer] Read MPU6050 register */
static uint8_t MPU6050_ReadReg(uint8_t reg)
{   
    /* Stage 1: Tell the MPU6050 "which register I want to operate on" (START, send address (write), send reg) */
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2C1, reg);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);
    /* Stage 2: Switch direction, Restart (issue START again, send the address again, direction changed to Receiver) */
/* 
    Restart: switching direction requires a fresh START. Reading a register is
    fundamentally a two-step process: first "write" the address of the
    register you want to read, then "read" that register's content. The
    direction changes from Transmitter to Receiver, which requires issuing
    a new START to switch direction.
*/
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    /* Stage 3: Since only 1 byte is being received, disable ACK early and queue up STOP early */
    /* Special timing for single-byte reads: must be handled ahead of waiting to receive */
    I2C_AcknowledgeConfig(I2C1, DISABLE);  // Disable ACK ahead of time
    I2C_GenerateSTOP(I2C1, ENABLE);        // Request STOP ahead of time
    /* Stage 4: Wait for the data to actually arrive, then read it out */
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);   // Wait for the data to actually arrive first (EV7)
    uint8_t Data = I2C_ReceiveData(I2C1);                // Then read it out
    /* Stage 5: Restore ACK enable (in preparation for possible future multi-byte reads) */
    I2C_AcknowledgeConfig(I2C1, ENABLE);   // Restore ACK, so it doesn't affect future multi-byte reads
    /* Stage 6: Return the value read */
    return Data;
}

/* [Driver layer] MPU6050 register configuration (device layer, specific to the MPU6050) */
static void MPU6050_Init(void)
{
    // SLEEP=0 (wake up), CYCLE=0 (don't use low-power cycle sampling), CLKSEL=001 (X-axis gyroscope as clock reference)
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    // All six axes (STBY_XA/YA/ZA/XG/YG/ZG) not in standby, keep all axes running
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    // Sample Rate = 1kHz / (1+4) = 200Hz, matches the IMU read task's scheduled frequency (200Hz)
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);
    // Bandwidth 21Hz, delay 8.5ms, EXT_SYNC_SET=000 (FSYNC not used)
    MPU6050_WriteReg(MPU6050_CONFIG, 0x04);
    // FS_SEL=2, ±1000°/s
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    // AFS_SEL=2, ±8g, ACCEL_HPF=0 (high-pass filter not used, doesn't affect the actual data registers read)
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);

}

/* [Driver layer] I2C/MPU6050 initialization (external interface) */
void I2C_MPU6050_Init(void)
{
    I2C_GPIO_Init();            //1. Get the pins ready
    I2C1_Protocol_Init();       //2. Get the I2C bus protocol in place
    MPU6050_Init();             //3. Finally configure the MPU6050-specific registers
}

/**
 * [Driver layer] Get MPU6050 device ID (external Get interface, used to verify communication)
 * GetID provides a clear way to verify: the value read back should equal the fixed value specified in the datasheet
 */
uint8_t MPU6050_GetID(void)
{   // Read the WHO_AM_I register (read-only, fixed factory value), verify the I2C communication link is working
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}


/**
 * [Driver layer] Get raw six-axis data (external Get interface)
 *
 * Reads a total of 6 axes across accelerometer/gyroscope, each axis 16 bits
 * (concatenated from H+L register pairs)
 *
 */
void MPU6050_GetData(MPU6050_Info* Data)
{
    uint8_t buf[14];
    
    // Send the starting register address 0x3B (AccX_H)
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);
    
    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    
    I2C_SendData(I2C1, 0x3B);  // AccX_H starting address
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    
    // Restart, switch to receive mode
    I2C_GenerateSTART(I2C1, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);
    
    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    
    // Read 14 bytes continuously
    for(int i = 0; i < 14; i++)
    {
        if(i == 13)  // Disable ACK before the last byte
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);
        buf[i] = I2C_ReceiveData(I2C1);
    }
    
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    
    // Assemble the data
    Data->AccX  = (int16_t)(buf[0]  << 8 | buf[1]);
    Data->AccY  = (int16_t)(buf[2]  << 8 | buf[3]);
    Data->AccZ  = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6]/buf[7] are temperature, skip
    Data->GyroX = (int16_t)(buf[8]  << 8 | buf[9]);
    Data->GyroY = (int16_t)(buf[10] << 8 | buf[11]);
    Data->GyroZ = (int16_t)(buf[12] << 8 | buf[13]);
}


