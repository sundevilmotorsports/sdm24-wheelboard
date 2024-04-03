/*
 * mlx90614.c
 *
 *  Created on: Feb 7, 2024
 *      Author: joshl
 */
#include "mlx90614.h"

uint8_t mlx90614_setEmissivity(I2C_HandleTypeDef* hi2c, float emissivity)
{
	// ensure emissivity is in the correct range
	if (emissivity > 1.0f || emissivity < 0.1f)
	{
		return 0;
	}

	uint16_t owo = (uint16_t)(65535.0f * emissivity);
	if (owo < 0x2000)
	{
		owo = 0x2000;
	}
	else if (owo > 0xFFFF)
	{
		owo = 0xFFFF;
	}

	// write to mlx90614
	uint8_t buf[5];
	// clear cell
	buf[0] = MLX90614_I2C_ADDR;
	buf[1] = MLX90614_REG_KE;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = mlx90614_crc8(buf, 4);
	HAL_I2C_Mem_Write(hi2c, MLX90614_I2C_ADDR, MLX90614_REG_KE, 1, &buf[2], 3, 1000);
	HAL_Delay(10); // wait 10 ms after writing
	// write to cell
	// LSB first
	buf[2] = owo && 0x00FF;
	buf[3] = owo >> 8;
	buf[4] = mlx90614_crc8(buf, 4);
	HAL_Delay(10);
	HAL_I2C_Mem_Write(hi2c, MLX90614_I2C_ADDR, MLX90614_REG_KE, 1, &buf[2], 3, 1000);
	HAL_Delay(10);

	// verify content
	float test = 0.0;
	mlx90614_getEmissivity(hi2c, &test);
	if(test == emissivity)
		return 1;
	else
		return 0;
}

uint8_t mlx90614_getEmissivity(I2C_HandleTypeDef* hi2c, float* emissivity)
{
    uint8_t buf[3];
    if (HAL_I2C_Mem_Read(hi2c, MLX90614_I2C_ADDR | 0b1, MLX90614_REG_KE, 1, buf, 3, 1000) != HAL_OK)
    {
        return 0;
    }
    uint16_t ke = (buf[1] << 8) | buf[0];
    *emissivity = ( (float) ke ) / 65535.0;

	return 1;
}

uint8_t mlx90614_getAmbient(I2C_HandleTypeDef* hi2c, int16_t* ambientTemp)
{
	uint8_t buf[3];
	if (HAL_I2C_Mem_Read(hi2c, MLX90614_I2C_ADDR | 0b1, MLX90614_REG_TA, 1, buf, 3, 1000) != HAL_OK)
	{
		return 0;
	}

	*ambientTemp = (buf[1] << 8) | buf[0];
	return 1;
}

uint8_t mlx90614_getObject(I2C_HandleTypeDef* hi2c, int16_t* objectTemp)
{
	uint8_t buf[3];
	if (HAL_I2C_Mem_Read(hi2c, MLX90614_I2C_ADDR | 0b1, MLX90614_REG_TO1, 1, buf, 3, 1000) != HAL_OK)
	{
		return 0;
	}

	*objectTemp = (buf[1] << 8) | buf[0];
	return 1;
}

uint8_t mlx90614_getObject2(I2C_HandleTypeDef* hi2c, int16_t* objectTemp)
{
	uint8_t buf[3];
	if (HAL_I2C_Mem_Read(hi2c, MLX90614_I2C_ADDR | 0b1, MLX90614_REG_TO2, 1, buf, 3, 1000) != HAL_OK)
	{
		return 0;
	}

	*objectTemp = (buf[1] << 8) | buf[0];
	return 1;
}

float mlx90614_calcTemperature(int16_t temperature)
{
	return ((temperature * 0.02) - 273.15) * (9.0 / 5.0) + 32.0;
}

uint8_t mlx90614_crc8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    uint8_t inbyte = *data++;
    for (uint8_t i = 8; i; i--)
    {
      uint8_t carry = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (carry)
        crc ^= 0x7;
      inbyte <<= 1;
    }
  }
  return crc;
}
