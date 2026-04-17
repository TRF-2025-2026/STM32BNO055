/*
 * eeprom.c
 *
 *  Created on: Apr 15, 2026
 *      Author: chait
 */
#include "eeprom.h"

#define ERASED          ((uint16_t)0xFFFF)
#define RECEIVE_DATA    ((uint16_t)0xEEEE)
#define VALID_PAGE      ((uint16_t)0x0000)

#define PAGE0_ID        FLASH_SECTOR_2
#define PAGE1_ID        FLASH_SECTOR_3
#define PAGE0_BASE      0x08008000
#define PAGE1_BASE      0x0800C000
#define PAGE_SIZE       16384

uint16_t EE_Init(void) {
    uint16_t Page0Status = (*(__IO uint16_t*)PAGE0_BASE);
    uint16_t Page1Status = (*(__IO uint16_t*)PAGE1_BASE);

    HAL_FLASH_Unlock();

    if (Page0Status == ERASED && Page1Status == ERASED) {
        FLASH_Erase_Sector(PAGE0_ID, VOLTAGE_RANGE_3);
        FLASH_Erase_Sector(PAGE1_ID, VOLTAGE_RANGE_3);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE0_BASE, VALID_PAGE);
    }
    else if (Page0Status == RECEIVE_DATA && Page1Status == VALID_PAGE) {
        FLASH_Erase_Sector(PAGE0_ID, VOLTAGE_RANGE_3);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE1_BASE, VALID_PAGE);
    }
    else if (Page0Status == VALID_PAGE && Page1Status == RECEIVE_DATA) {
        FLASH_Erase_Sector(PAGE1_ID, VOLTAGE_RANGE_3);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, PAGE0_BASE, VALID_PAGE);
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data) {
    uint32_t PageBase = PAGE0_BASE;
    if ((*(__IO uint16_t*)PAGE1_BASE) == VALID_PAGE) {
        PageBase = PAGE1_BASE;
    }

    uint32_t Address = PageBase + PAGE_SIZE - 4;

    while (Address > (PageBase + 2)) {
        uint32_t Val = (*(__IO uint32_t*)Address);
        if ((uint16_t)(Val >> 16) == VirtAddress) {
            *Data = (uint16_t)(Val & 0xFFFF);
            return 0;
        }
        Address -= 4;
    }
    return 1;
}

uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data) {
    uint32_t PageBase = PAGE0_BASE;
    if ((*(__IO uint16_t*)PAGE1_BASE) == VALID_PAGE) {
        PageBase = PAGE1_BASE;
    }

    uint32_t TargetAddress = PageBase + 4;
    while (TargetAddress < (PageBase + PAGE_SIZE)) {
        if ((*(__IO uint32_t*)TargetAddress) == 0xFFFFFFFF) {
            break;
        }
        TargetAddress += 4;
    }

    if (TargetAddress >= (PageBase + PAGE_SIZE)) {
        return 0x0080;
    }

    HAL_FLASH_Unlock();
    uint32_t WriteValue = (uint32_t)(VirtAddress << 16) | Data;
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TargetAddress, WriteValue);
    HAL_FLASH_Lock();

    return (uint16_t)status;
}
