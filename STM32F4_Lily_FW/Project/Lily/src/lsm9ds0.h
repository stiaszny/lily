#ifndef __LSM9DS0_H
#define __LSM9DS0_H

#include <stdint.h>
#include <stdbool.h>

#define SA_MAG  0x3C
#define SA_GYRO 0xD4

#define REG_ADDR_DATA_ACCEL    0x28
#define REG_ADDR_DATA_MAG      0x08
#define REG_ADDR_DATA_GYRO     0x28
#define REG_ADDR_STATUS_ACCEL  0x27
#define REG_ADDR_STATUS_MAG    0x07
#define REG_ADDR_STATUS_GYRO   0x27

#define DATA_AVAILABLE_MASK (1 << 3)

typedef enum {
  MAG,
  ACCEL,
  GYRO
} SensorType;

void lsmRegRead(uint8_t i2cAddr, uint8_t regAddr, uint16_t numBytes, uint8_t *result);
void lsmReadMag(uint8_t *result);
void lsmReadAccel(uint8_t *result);
void lsmReadGyro(uint8_t *result);
bool isDataReady(SensorType sensor);

#endif
