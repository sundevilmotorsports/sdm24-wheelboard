#include "eeprom.h"

HAL_StatusTypeDef eeprom_read(I2C_HandleTypeDef* hi2c, uint8_t block, uint8_t addr, uint8_t* data)
{
	uint16_t device_addr = EEPROM_I2C_ADDR | ((block & 0b111) << 1) | 0b1;
	return HAL_I2C_Mem_Read(hi2c, device_addr, addr, 1, data, 1, 1000);
}

HAL_StatusTypeDef eeprom_write(I2C_HandleTypeDef* hi2c, uint8_t block, uint8_t addr, uint8_t* data)
{
	// create device address: 0b1010[3 bit block addr][r/w]
	uint16_t device_addr = EEPROM_I2C_ADDR | ((block & 0b111) << 1) | 0b0;
	return HAL_I2C_Mem_Write(hi2c, device_addr, addr, 1, data, 1, 1000);
}
