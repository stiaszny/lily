/* i2c driver for the STM LSM9SD0 9 DOF IMU */
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "lsm9ds0.h"

#define I2C_PERIPH  I2C1

/* at least the interface ST presents in STM32Cube is much more refined. */
void lsmRegRead(uint8_t i2cAddr, uint8_t regAddr, uint16_t numBytes, uint8_t *result)
{
    int count = 0;              /* track number of cycles wasted in spin-wait */
    int i;

    I2C_AcknowledgeConfig(I2C_PERIPH, ENABLE);         /* enable Master ACK */

    I2C_GenerateSTART(I2C_PERIPH, ENABLE);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) {
        count++;                /* would be nice to get these running on interrupt... */
    }

    I2C_Send7bitAddress(I2C_PERIPH, i2cAddr, I2C_Direction_Transmitter);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS) {
        count++;
    }

    if (numBytes > 1) {
        regAddr |= 0x80;        /* enable address auto-increment */
    }
    /* where is event EV8_1, as defined by the reference manual? */
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

    if (numBytes == 1) {
        I2C_AcknowledgeConfig(I2C_PERIPH, DISABLE);
        while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS) {
            count++;
        }
        I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
        result[0] = I2C_ReceiveData(I2C_PERIPH);
    }

    if (numBytes == 2) {
        /* according the ref. man. POS should be set HIGH (and ACK
           disabled) before clearing ADDR, which CheckEvent will do
           inherently.  Lovely. Let's see if it works... */
        I2C_AcknowledgeConfig(I2C_PERIPH, DISABLE);
        I2C_NACKPositionConfig(I2C_PERIPH, I2C_NACKPosition_Next);

        while (I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_BTF) != SET) {
            count++;
        }
        I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
        result[0] = I2C_ReceiveData(I2C_PERIPH);
        result[1] = I2C_ReceiveData(I2C_PERIPH);
        I2C_NACKPositionConfig(I2C_PERIPH, I2C_NACKPosition_Current);
    }

    if (numBytes > 2) {
        for ( i = 0; i < numBytes; i++) {
            if ( i < numBytes - 3) {
                while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS) {
                    count++;
                }
                result[i] = I2C_ReceiveData(I2C_PERIPH);
            }
            if ( i == numBytes - 3) {
                while (I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_BTF) != SET) {
                    count++;
                }
                I2C_AcknowledgeConfig(I2C_PERIPH, DISABLE);
                result[i++] = I2C_ReceiveData(I2C_PERIPH);

                while (I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_BTF) != SET) {
                    count++;
                }
                I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
                result[i++] = I2C_ReceiveData(I2C_PERIPH);
                result[i++] = I2C_ReceiveData(I2C_PERIPH);
            }
        }
    }
    return;
}

void lsmReadMag(uint8_t *result)
{
    lsmRegRead(SA_MAG, REG_ADDR_DATA_MAG, 6, result);
}

void lsmReadAccel(uint8_t *result)
{
    lsmRegRead(SA_MAG, REG_ADDR_DATA_ACCEL, 6, result); 
}

void lsmReadGyro(uint8_t *result)
{
    lsmRegRead(SA_MAG, REG_ADDR_DATA_ACCEL, 6, result); 
}

bool isDataReady(SensorType sensor)
{
    uint8_t val;
    switch(sensor) {
    case MAG:
        lsmRegRead(SA_MAG, REG_ADDR_STATUS_MAG, 1, &val);
        break;
    case ACCEL:
        /* accel & mag on same addr - not copypasta! */
        lsmRegRead(SA_MAG, REG_ADDR_STATUS_ACCEL, 1, &val);
        break;
    case GYRO:
        lsmRegRead(SA_GYRO, REG_ADDR_STATUS_GYRO, 1, &val);
        break;
    }

    if ((val & DATA_AVAILABLE_MASK) != 0) 
        return true;
    else
        return false;
}
