/**
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "vl53l1_platform.h"

#include "stm32f1xx_hal.h"
#include "main.h"

#define I2C_TIME_OUT_BASE 10
#define I2C_TIME_OUT_BYTE 1

uint8_t _I2CBuffer[256];
I2C_HandleTypeDef *VL53L1_i2c_handle = &hi2c1;

static uint8_t _I2CWrite(uint16_t dev, uint8_t *pdata, uint32_t count)
{
  uint8_t status;
  uint32_t i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

  status = HAL_I2C_Master_Transmit(VL53L1_i2c_handle, dev, pdata, count, i2c_time_out);

  return status;
}

static uint8_t _I2CRead(uint16_t dev, uint8_t *pdata, uint32_t count)
{
  uint8_t status;
  uint32_t i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

  status = HAL_I2C_Master_Receive(VL53L1_i2c_handle, dev | 1, pdata, count, i2c_time_out);

  return status;
}

static void VL53L1_GetI2cBus()
{
  return; // to be implemented
}

static void VL53L1_PutI2cBus()
{
  return; // to be implemented
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  int8_t ret = 0;

  if (count > sizeof(_I2CBuffer) - 1)
  {
    return 1;
  }
  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;
  memcpy(&_I2CBuffer[2], pdata, count);

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, count + 2);
  VL53L1_PutI2cBus();

  return ret;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  int8_t ret = 0;

  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, 2);
  if (ret != 0)
    goto done;

  ret = _I2CRead(dev, pdata, count);

done:
  VL53L1_PutI2cBus();
  return ret;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
  int8_t ret = 0;

  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;
  _I2CBuffer[2] = data;

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, 3);

  VL53L1_PutI2cBus();
  return ret;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
  int8_t ret = 0;

  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;
  _I2CBuffer[2] = data >> 8;
  _I2CBuffer[3] = data & 0x00FF;

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, 4);

  VL53L1_PutI2cBus();
  return ret;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  int8_t ret = 0;

  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;
  _I2CBuffer[2] = (data >> 24) & 0xFF;
  _I2CBuffer[3] = (data >> 16) & 0xFF;
  _I2CBuffer[4] = (data >> 8) & 0xFF;
  _I2CBuffer[5] = (data >> 0) & 0xFF;

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, 6);

  VL53L1_PutI2cBus();
  return ret;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
  int8_t ret = 0;

  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, 2);
  if (ret)
    goto done;

  ret = _I2CRead(dev, data, 1);

done:
  VL53L1_PutI2cBus();
  return ret;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
  int8_t ret = 0;

  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, 2);
  if (ret)
    goto done;

  ret = _I2CRead(dev, _I2CBuffer, 2);
  if (ret)
    goto done;

  *data = ((uint16_t)_I2CBuffer[0] << 8) + (uint16_t)_I2CBuffer[1];

done:
  VL53L1_PutI2cBus();
  return ret;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
  int8_t ret = 0;

  _I2CBuffer[0] = index >> 8;
  _I2CBuffer[1] = index & 0xFF;

  VL53L1_GetI2cBus();
  ret = _I2CWrite(dev, _I2CBuffer, 2);
  if (ret != 0)
    goto done;

  ret = _I2CRead(dev, _I2CBuffer, 4);
  goto done;

  *data = ((uint32_t)_I2CBuffer[0] << 24) + ((uint32_t)_I2CBuffer[1] << 16) + ((uint32_t)_I2CBuffer[2] << 8) + (uint32_t)_I2CBuffer[3];

done:
  VL53L1_PutI2cBus();
  return ret;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
  (void)dev;
  HAL_Delay(wait_ms);
  return 0;
}
