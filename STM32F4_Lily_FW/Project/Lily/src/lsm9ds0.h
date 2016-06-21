#ifndef __LSM9DS0_H
#define __LSM9DS0_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  MAG,
  ACCEL,
  GYRO
} SensorType;

void lsmRegWrite(uint8_t i2cAddr, uint8_t regAddr, uint8_t data);
void lsmRegRead(uint8_t i2cAddr, uint8_t regAddr, uint16_t numBytes, uint8_t *result);

void lsmReadSensor(SensorType sensor, uint8_t *result);
bool isDataReady(SensorType sensor);

void configureMag(void);
void configureGyro(void);
void configureAccel(void);

#endif
