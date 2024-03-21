/*
 * mlx90614.c
 *
 *  Created on: Feb 7, 2024
 *      Author: joshl
 */
#include "mlx90614.h"

uint8_t mlx90614_setEmissivity(I2C_HandleTypeDef* hi2c, float emissivity)
{
	if (emissivity > 1.0f || emissivity < 0.1f)
	{
		return 0;
	}

	uint16_t owo = (uint16_t)(65535.0f * emissivity);
	if (owo < 0x2000)
	{
		owo = 0x2000;
	}


	return 1;
}

uint8_t mlx90614_getEmissivity(I2C_HandleTypeDef* hi2c, float* emissivity)
{
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
