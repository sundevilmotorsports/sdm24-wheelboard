/*
 * mlx90614.h
 *
 *  Created on: Feb 7, 2024
 *      Author: joshl
 */

#ifndef INC_MLX90614_H_
#define INC_MLX90614_H_

#include "main.h"

#define MLX90614_I2C_ADDR 	(0x5A << 1)
#define MLX90614_REG_TA 	0x06
#define MLX90614_REG_TO1 	0x07
#define MLX90614_REG_TO2 	0x08
#define MLX90614_REG_KE  	0x24

uint8_t mlx90614_setEmissivity(I2C_HandleTypeDef* hi2c, float emissivity);
uint8_t mlx90614_getEmissivity(I2C_HandleTypeDef* hi2c, float* emissivity);
uint8_t mlx90614_getAmbient(I2C_HandleTypeDef* hi2c, int16_t* ambientTemp);
uint8_t mlx90614_getObject(I2C_HandleTypeDef* hi2c, int16_t* objectTemp);
uint8_t mlx90614_getObject2(I2C_HandleTypeDef* hi2c, int16_t* objectTemp);
float mlx90614_calcTemperature(int16_t temperature);
uint8_t mlx90614_crc8(uint8_t *data, uint8_t len);

#endif /* INC_MLX90614_H_ */
