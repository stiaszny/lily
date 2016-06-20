#ifndef __LSM9DS0_H
#define __LSM9DS0_H

void lsmRegRead(uint8_t i2cAddr, uint8_t regAddr, uint16_t numBytes, uint8_t *result);

#endif
