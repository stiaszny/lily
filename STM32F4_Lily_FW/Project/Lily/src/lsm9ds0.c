/* i2c driver for the STM LSM9SD0 9 DOF IMU */
#include <stdint.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#define I2C_PERIPH  I2C1

uint16_t lsmRegReadMulti(uint8_t i2cAddr, uint8_t regAddr, uint16_t numBytes, uint8_t *result)
{
    int bytesRead = 0;
    return bytesRead;
}

uint8_t lsmRegReadSingle(uint8_t i2cAddr, uint8_t regAddr)
{
    int count = 0;              /* track number of cycles wasted in spin-wait */

    I2C_GenerateSTART(I2C_PERIPH, ENABLE);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) {
        count++;                /* would be nice to get these running on interrupt... */
    }

    I2C_Send7bitAddress(I2C_PERIPH, i2cAddr, I2C_Direction_Transmitter);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS) {
        count++;
    }

    I2C_SendData(I2C_PERIPH, regAddr);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) {
        count++;
    }

    I2C_GenerateSTART(I2C_PERIPH, ENABLE);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) {
        count++;
    }

    I2C_Send7bitAddress(I2C_PERIPH, i2cAddr, I2C_Direction_Receiver);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS) {
        count++;
    }
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS) {
        count++;
    }

    return I2C_ReceiveData(I2C_PERIPH);
}

/* caller is repsonsible for properly allocating result */
uint16_t lsmRegRead(uint8_t i2cAddr, uint8_t regAddr, uint16_t numBytes, uint8_t *result)
{
    if (numBytes == 1) {
        *result = lsmRegReadSingle(i2cAddr, regAddr);
        return 1;
    } else {
        return lsmRegReadMulti(i2cAddr, regAddr, numBytes, result);
    }
}


//lsmReadMag();
//lsmReadAccel();
//lsmReadGyro();
