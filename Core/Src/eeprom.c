#include "eeprom.h"

HAL_StatusTypeDef eeprom_read(I2C_HandleTypeDef* hi2c, uint8_t block, uint8_t addr, uint8_t* data)
{
	uint16_t device_addr = EEPROM_I2C_ADDR | ((block & 0b111) << 1) | 0b1;
	return HAL_I2C_Mem_Read(hi2c, device_addr, addr, 1, data, 1, EEPROM_TIMEOUT);
}

HAL_StatusTypeDef eeprom_write(I2C_HandleTypeDef* hi2c, uint8_t block, uint8_t addr, uint8_t* data)
{
	// create device address: 0b1010[3 bit block addr][r/w]
	uint16_t device_addr = EEPROM_I2C_ADDR | ((block & 0b111) << 1) | 0b0;
	return HAL_I2C_Mem_Write(hi2c, device_addr, addr, 1, data, 1, 1000);
}

HAL_StatusTypeDef eeprom_config_read(I2C_HandleTypeDef* hi2c, uint16_t* can_addr)
{
    uint8_t temp_addr_hi;
    uint8_t temp_addr_lo;
    eeprom_read(hi2c, 0b000, CAN_ADDR0, &temp_addr_hi);
    HAL_Delay(1);
    eeprom_read(hi2c, 0b000, CAN_ADDR1, &temp_addr_lo);
    *can_addr = (temp_addr_hi << 8) | (temp_addr_lo);
    return HAL_OK;
}

HAL_StatusTypeDef eeprom_config_write(I2C_HandleTypeDef* hi2c, uint16_t can_addr)
{
    uint8_t addr_hi = (can_addr >> 8) & 0xFF;
    uint8_t addr_lo = can_addr & 0xFF;
    eeprom_write(hi2c, 0b000, CAN_ADDR0, &addr_hi);
    HAL_Delay(1);
    eeprom_write(hi2c, 0b000, CAN_ADDR1, &addr_lo);
    return HAL_OK;
}

