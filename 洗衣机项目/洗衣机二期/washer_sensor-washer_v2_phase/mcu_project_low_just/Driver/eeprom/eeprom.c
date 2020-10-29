/**********************************************************************
 * @File        : eeprom.c
 * @Brief  	    : 
 * @ChangeLogs  :
 * Date           Author       Notes
 * 2020-8-12       xiao          the first version
 **********************************************************************/
#include "eeprom.h"
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;
__IO TestStatus DataMemoryProgramStatus = PASSED;
uint32_t NbrOfPage = 0, j = 0, Address = 0;
uint32_t eeprom_data[3] = {0};
/************************************************************************
 * @brief  	向EEPROM中写入可指定长度的整型数据
 * @param  	uint32_t *Data：待写入数据起始地址
 *          uint8_t len：待写入的长度
 * @retval 	None	
 * 
 ************************************************************************/
void eeprom_write_word(uint32_t *Data,uint8_t len)
{
	int i = 0;
	for(i=0;i<len;i++)
	{
		Eeprom_Write_Byte((4*i),Data[i]);
	}
}
/************************************************************************
 * @brief  	擦除EEPROM中的数据
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void eeprom_erase_memory(void)
{
	Address = DATA_EEPROM_START_ADDR;
  DATA_EEPROM_EraseWord(Address);
	DATA_EEPROM_EraseWord(Address+4);
	DATA_EEPROM_EraseWord(Address+8);
}
/************************************************************************
 * @brief  	从EEPROM中读出指定长度的数据
 * @param  	uint32_t *buf：存放读出数据的缓冲区起始地址
 *          uint8_t len：读出的数据个数
 * @retval 	None	
 * 
 ************************************************************************/
void eeprom_read_word(uint32_t *buf,uint8_t len)
{
	int i = 0;
	for(i=0;i<len;i++)
	{
		buf[i] = Eeprom_Read_Byte(i*4);
	}
}
/************************************************************************
 * @brief  	向EEPROM指定地址中写入一个整型数据
 * @param  	uint32_t Start_Address：待写入的地址
 *          uint32_t Data_Value：待写入的数据
 * @retval 	None	
 * 
 ************************************************************************/
void Eeprom_Write_Byte(uint32_t Start_Address,uint32_t Data_Value)
{

   uint32_t Address =0;
   Address = DATA_EEPROM_START_ADDR+Start_Address;
   if(IS_FLASH_DATA_ADDRESS(Address))
   {
		 /*设备解锁*/
     DATA_EEPROM_Unlock();  
		 FLASH_ClearFlag(FLASH_FLAG_BSY  |FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | 
		 FLASH_ERROR_PROGRAM| FLASH_ERROR_PROGRAM| FLASH_FLAG_ENDHV|FLASH_FLAG_READY |FLASH_FLAG_SIZERR |
		 FLASH_FLAG_OPTVERR| FLASH_FLAG_OPTVERRUSR);
     while(FLASH_GetStatus()!=FLASH_COMPLETE);
     DATA_EEPROM_ProgramWord(Address,Data_Value);
		 /*设备上锁*/
     DATA_EEPROM_Lock();
   }
}
/************************************************************************
 * @brief  	从EEPROM指定地址中读出一个整型数据
 * @param  	uint32_t Read_Address：待读出的地址
 * @retval 	None	
 * 
 ************************************************************************/
uint32_t Eeprom_Read_Byte(uint32_t Read_Address)
{
    uint32_t Address =0;
    Address = DATA_EEPROM_START_ADDR+Read_Address;
    uint32_t tmp=0;
    if(IS_FLASH_DATA_ADDRESS(Address))
    {
				/*设备解锁*/
        DATA_EEPROM_Unlock();  
				FLASH_ClearFlag(FLASH_FLAG_BSY  |FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | 
				FLASH_ERROR_PROGRAM| FLASH_ERROR_PROGRAM| FLASH_FLAG_ENDHV|FLASH_FLAG_READY |FLASH_FLAG_SIZERR |
				FLASH_FLAG_OPTVERR| FLASH_FLAG_OPTVERRUSR);
        while(FLASH_GetStatus()==FLASH_BUSY);
        tmp=*(__IO uint32_t*)Address;
				/*设备上锁*/
        DATA_EEPROM_Lock();
    }
    return tmp;
}
