/**********************************************************************
 * @File        : eeprom.h
 * @Brief  	    : 
 * @ChangeLogs  :
 * Date           Author       Notes
 * 2020-8-12       xiao          the first version
 **********************************************************************/
#ifndef _EEPROM_H_
#define _EEPROM_H_
#include "stm32l1xx.h"
#include "stm32l1xx_flash.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x08081FFF
#define DATA_EEPROM_PAGE_SIZE      0x8
//#define DATA_32                    0x12345678
//#define FAST_DATA_32               0x55667799

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO FLASH_Status FLASHStatus;
extern __IO TestStatus DataMemoryProgramStatus;
extern uint32_t NbrOfPage,Address;
extern uint32_t eeprom_data[3];
  
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void eeprom_write_word(uint32_t *Data,uint8_t len);
void eeprom_erase_memory(void);
void eeprom_read_word(uint32_t *buf,uint8_t len);
void Eeprom_Write_Byte(uint32_t Start_Address,uint32_t Data_Value);
uint32_t Eeprom_Read_Byte(uint32_t Read_Address);
#endif
