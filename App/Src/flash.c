#include "flash.h"
#include "stm32f4xx_hal_flash_ex.h"

const uint32_t ADDR_FLASH_SECTOR[24] = {
    ((uint32_t)0x08000000), /* 0 */
    ((uint32_t)0x08004000), /* 1 */
    ((uint32_t)0x08008000), /* 2 */
    ((uint32_t)0x0800C000), /* 3 */

    ((uint32_t)0x08010000), /* 4 */
    ((uint32_t)0x08020000), /* 5 */
    ((uint32_t)0x08040000), /* 6 */
    ((uint32_t)0x08060000), /* 7 */
    ((uint32_t)0x08080000), /* 8 */
    ((uint32_t)0x080A0000), /* 9 */
    ((uint32_t)0x080C0000), /* 10 */
    ((uint32_t)0x080E0000), /* 11 */
    ((uint32_t)0x08100000), /* 12 */
    ((uint32_t)0x08104000), /* 13 */
    ((uint32_t)0x08108000), /* 14 */
    ((uint32_t)0x0810C000), /* 15 */
    /*2*/
    ((uint32_t)0x08110000), /* 16 */
    ((uint32_t)0x08120000), /* 17 */
    ((uint32_t)0x08140000), /* 18 */
    ((uint32_t)0x08160000), /* 19 */
    ((uint32_t)0x08180000), /* 20 */
    ((uint32_t)0x081A0000), /* 21 */
    ((uint32_t)0x081C0000), /* 22 */
    ((uint32_t)0x081E0000), /* 23 */
};

uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;
    if ((Address < ADDR_FLASH_SECTOR[1]) && (Address >= ADDR_FLASH_SECTOR[0])) {
        sector = FLASH_SECTOR_0;
    } else if ((Address < ADDR_FLASH_SECTOR[2]) && (Address >= ADDR_FLASH_SECTOR[1])) {
        sector = FLASH_SECTOR_1;
    } else if ((Address < ADDR_FLASH_SECTOR[3]) && (Address >= ADDR_FLASH_SECTOR[2])) {
        sector = FLASH_SECTOR_2;
    } else if ((Address < ADDR_FLASH_SECTOR[4]) && (Address >= ADDR_FLASH_SECTOR[3])) {
        sector = FLASH_SECTOR_3;
    } else if ((Address < ADDR_FLASH_SECTOR[5]) && (Address >= ADDR_FLASH_SECTOR[4])) {
        sector = FLASH_SECTOR_4;
    } else if ((Address < ADDR_FLASH_SECTOR[6]) && (Address >= ADDR_FLASH_SECTOR[5])) {
        sector = FLASH_SECTOR_5;
    } else if ((Address < ADDR_FLASH_SECTOR[7]) && (Address >= ADDR_FLASH_SECTOR[6])) {
        sector = FLASH_SECTOR_6;
    } else if ((Address < ADDR_FLASH_SECTOR[8]) && (Address >= ADDR_FLASH_SECTOR[7])) {
        sector = FLASH_SECTOR_7;
    } else if ((Address < ADDR_FLASH_SECTOR[9]) && (Address >= ADDR_FLASH_SECTOR[8])) {
        sector = FLASH_SECTOR_8;
    } else if ((Address < ADDR_FLASH_SECTOR[10]) && (Address >= ADDR_FLASH_SECTOR[9])) {
        sector = FLASH_SECTOR_9;
    } else if ((Address < ADDR_FLASH_SECTOR[11]) && (Address >= ADDR_FLASH_SECTOR[10])) {
        sector = FLASH_SECTOR_10;
    } else if ((Address < ADDR_FLASH_SECTOR[12]) && (Address >= ADDR_FLASH_SECTOR[11])) {
        sector = FLASH_SECTOR_11;
    }

#if 0 
    else if ((Address < ADDR_FLASH_SECTOR[13]) && (Address >= ADDR_FLASH_SECTOR[12])) {
        sector = FLASH_SECTOR_12;
    } else if ((Address < ADDR_FLASH_SECTOR[14]) && (Address >= ADDR_FLASH_SECTOR[13])) {
        sector = FLASH_SECTOR_13;
    } else if ((Address < ADDR_FLASH_SECTOR[15]) && (Address >= ADDR_FLASH_SECTOR[14])) {
        sector = FLASH_SECTOR_14;
    } else if ((Address < ADDR_FLASH_SECTOR[16]) && (Address >= ADDR_FLASH_SECTOR[15])) {
        sector = FLASH_SECTOR_15;
    } else if ((Address < ADDR_FLASH_SECTOR[17]) && (Address >= ADDR_FLASH_SECTOR[16])) {
        sector = FLASH_SECTOR_16;
    } else if ((Address < ADDR_FLASH_SECTOR[18]) && (Address >= ADDR_FLASH_SECTOR[17])) {
        sector = FLASH_SECTOR_17;
    } else if ((Address < ADDR_FLASH_SECTOR[19]) && (Address >= ADDR_FLASH_SECTOR[18])) {
        sector = FLASH_SECTOR_18;
    } else if ((Address < ADDR_FLASH_SECTOR[20]) && (Address >= ADDR_FLASH_SECTOR[19])) {
        sector = FLASH_SECTOR_19;
    } else if ((Address < ADDR_FLASH_SECTOR[21]) && (Address >= ADDR_FLASH_SECTOR[20])) {
        sector = FLASH_SECTOR_20;
    } else if ((Address < ADDR_FLASH_SECTOR[22]) && (Address >= ADDR_FLASH_SECTOR[21])) {
        sector = FLASH_SECTOR_21;
    } else if ((Address < ADDR_FLASH_SECTOR[23]) && (Address >= ADDR_FLASH_SECTOR[22])) {
        sector = FLASH_SECTOR_22;
    } else if (Address >= ADDR_FLASH_SECTOR[23]) {
        sector = FLASH_SECTOR_23;
    }
#endif
    return sector;
}
void Flash_Write8(uint32_t FLASH_USER_WRITE_ADDR, uint8_t* DATA_8, uint8_t len)
{
    FLASH_EraseInitTypeDef ers;
    ers.TypeErase = FLASH_TYPEERASE_SECTORS;
    ers.Sector = GetSector(FLASH_USER_WRITE_ADDR);
    ers.NbSectors = 1;
    ers.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t errSec = 0;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    HAL_FLASHEx_Erase(&ers, &errSec);

    for (uint8_t i = 0; i < len; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_USER_WRITE_ADDR + i, (uint64_t)(*(DATA_8 + i)));
    }
    HAL_FLASH_Lock();
}

void Flash_Read8(uint32_t FLASH_USER_READ_ADDR, uint8_t* Data_8, uint8_t len)
{
    uint8_t i;
    uint32_t Address = FLASH_USER_READ_ADDR;
    HAL_FLASH_Lock();
    for (i = 0; i < len; i++) {
        *Data_8 = *(volatile uint32_t*)Address;
        Address++;
        Data_8++;
    }
}

void Flash_Write32(uint32_t FLASH_USER_WRITE_ADDR, uint32_t* DATA_32, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_USER_WRITE_ADDR + i / 4, (uint64_t)(DATA_32 + i));
    }
}

void Flash_Read32(uint32_t FLASH_USER_READ_ADDR, uint32_t* Data_32, uint8_t len)
{
    uint8_t i;
    uint32_t Address = FLASH_USER_READ_ADDR;
    HAL_FLASH_Lock();
    for (i = 0; i < len; i++) {
        *Data_32 = *(volatile uint32_t*)Address;
        Address += 4;
        Data_32++;
    }
}
