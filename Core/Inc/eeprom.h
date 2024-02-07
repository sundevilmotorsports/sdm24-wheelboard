#pragma once

#include "main.h"

#define EEPROM_I2C_ADDR (0b1010 << 4)

HAL_StatusTypeDef eeprom_read(I2C_HandleTypeDef* hi2c, uint8_t block, uint8_t addr, uint8_t* data);
HAL_StatusTypeDef eeprom_write(I2C_HandleTypeDef* hi2c, uint8_t block, uint8_t addr, uint8_t* data);
