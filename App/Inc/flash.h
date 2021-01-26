#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

extern const uint32_t ADDR_FLASH_SECTOR[24];

/*外部接口函数*/
uint32_t GetSector(uint32_t Address);
void Flash_Write8(uint32_t FLASH_USER_WRITE_ADDR, uint8_t* DATA_8, uint8_t len);
void Flash_Read8(uint32_t FLASH_USER_READ_ADDR, uint8_t* Data_8, uint8_t len);
void Flash_Write32(uint32_t FLASH_USER_WRITE_ADDR, uint32_t* DATA_32, uint8_t len);
void Flash_Read32(uint32_t FLASH_USER_READ_ADDR, uint32_t* Data_32, uint8_t len);
#endif
