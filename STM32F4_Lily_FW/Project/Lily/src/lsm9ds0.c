/* i2c driver for the STM LSM9SD0 9 DOF IMU */
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "stm32f4_discovery.h"
#include "my_printf.h"

#include "lsm9ds0.h"


#define I2C_PERIPH  I2C1

/* I know you love your static const ints, Nic :)  I'd fix this in production code. */
#define CR1          0x20
#define CR3          0x22
#define CR4          0x23
#define CR5          0x24
#define CR7_MAG      0x26
#define INT_CTRL_M   0x12

#define SA_MAG  0x3C
#define SA_GYRO 0xD4

#define REG_ADDR_DATA_ACCEL    0x28
#define REG_ADDR_DATA_MAG      0x08
#define REG_ADDR_DATA_GYRO     0x28
#define REG_ADDR_STATUS_ACCEL  0x27
#define REG_ADDR_STATUS_MAG    0x07
#define REG_ADDR_STATUS_GYRO   0x27

#define DATA_AVAILABLE_MASK (1 << 3)

extern bool accelDataReady;
extern bool magDataReady;

static void configurePA2(void);
static void configurePA3(void);

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

/* on this chip, there seems to be no need to ever write more than one byte */
void lsmRegWrite(uint8_t i2cAddr, uint8_t regAddr, uint8_t data)
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
    I2C_SendData(I2C_PERIPH, data);
    while (I2C_CheckEvent(I2C_PERIPH, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS) {
        count++;
    }
    I2C_GenerateSTOP(I2C_PERIPH, ENABLE);
}

void lsmReadSensor(SensorType sensor, uint8_t *result)
{
    switch (sensor) {
    case ACCEL:
        lsmRegRead(SA_MAG, REG_ADDR_DATA_ACCEL, 6, result);
        break;
    case GYRO:
        lsmRegRead(SA_GYRO, REG_ADDR_DATA_GYRO, 6, result);
        break;
    case MAG:
        lsmRegRead(SA_MAG, REG_ADDR_DATA_MAG, 6, result);
        break;
    }
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


/* of course, in a real driver, I would make these nice and take some arguments. */
void configureMag(void)
{
    uint8_t data[6];

    configurePA3();
    lsmRegWrite(SA_MAG, CR3, 0x2);    /* fire data ready interrupt on int1_xm */
    lsmRegWrite(SA_MAG, CR5, 0x10);   /* set rate */
    lsmRegWrite(SA_MAG, CR7_MAG, 0);  /* turn on */
    lsmReadSensor(MAG, data);         /* clear the data ready interrupt, if set */
}

void configureGyro(void)
{
    uint8_t data[6];

    lsmRegWrite(SA_GYRO, CR3, 0x8);  /* in theory, put data ready signal on DRDY_G pin  */
    lsmRegWrite(SA_GYRO, CR1, 0x3F); /* set rate, turn on. */
    lsmReadSensor(GYRO, data);       /* clear the data ready interrupt, if set */
}

void configureAccel(void)
{
    uint8_t data[6] = {0};

    configurePA2();
    lsmRegWrite(SA_MAG, INT_CTRL_M, 0x8);        /* default value in the datasheet is totally wrong */
    lsmRegWrite(SA_MAG, CR4, 0x8);               /* data ready on int2_xm pin */
    lsmRegWrite(SA_MAG, CR1, 0x17);              /* rate and axes enable */
    lsmReadSensor(ACCEL, data);                  /* reset the interrupt */
}

static void configurePA2(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);

    EXTI_InitStruct.EXTI_Line = EXTI_Line2;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

static void configurePA3(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);

    EXTI_InitStruct.EXTI_Line = EXTI_Line3;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01; /* MAG is lower priority than ACCEL */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/* Handle Accel (PA2) interrupt */
void EXTI2_IRQHandler(void) {
    /* should only need to check on multiplex interrupts, but can't hurt */
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
        accelDataReady = true;
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

/* Handle MAG (PA3) interrupt */
void EXTI3_IRQHandler(void) {
    /* should only need to check on multiplex interrupts, but can't hurt */
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        magDataReady = true;
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
