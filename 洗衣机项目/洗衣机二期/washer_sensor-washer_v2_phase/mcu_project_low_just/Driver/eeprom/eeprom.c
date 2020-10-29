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
 * @brief  	��EEPROM��д���ָ�����ȵ���������
 * @param  	uint32_t *Data����д��������ʼ��ַ
 *          uint8_t len����д��ĳ���
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
 * @brief  	����EEPROM�е�����
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
 * @brief  	��EEPROM�ж���ָ�����ȵ�����
 * @param  	uint32_t *buf����Ŷ������ݵĻ�������ʼ��ַ
 *          uint8_t len�����������ݸ���
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
 * @brief  	��EEPROMָ����ַ��д��һ����������
 * @param  	uint32_t Start_Address����д��ĵ�ַ
 *          uint32_t Data_Value����д�������
 * @retval 	None	
 * 
 ************************************************************************/
void Eeprom_Write_Byte(uint32_t Start_Address,uint32_t Data_Value)
{

   uint32_t Address =0;
   Address = DATA_EEPROM_START_ADDR+Start_Address;
   if(IS_FLASH_DATA_ADDRESS(Address))
   {
		 /*�豸����*/
     DATA_EEPROM_Unlock();  
		 FLASH_ClearFlag(FLASH_FLAG_BSY  |FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | 
		 FLASH_ERROR_PROGRAM| FLASH_ERROR_PROGRAM| FLASH_FLAG_ENDHV|FLASH_FLAG_READY |FLASH_FLAG_SIZERR |
		 FLASH_FLAG_OPTVERR| FLASH_FLAG_OPTVERRUSR);
     while(FLASH_GetStatus()!=FLASH_COMPLETE);
     DATA_EEPROM_ProgramWord(Address,Data_Value);
		 /*�豸����*/
     DATA_EEPROM_Lock();
   }
}
/************************************************************************
 * @brief  	��EEPROMָ����ַ�ж���һ����������
 * @param  	uint32_t Read_Address���������ĵ�ַ
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
				/*�豸����*/
        DATA_EEPROM_Unlock();  
				FLASH_ClearFlag(FLASH_FLAG_BSY  |FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | 
				FLASH_ERROR_PROGRAM| FLASH_ERROR_PROGRAM| FLASH_FLAG_ENDHV|FLASH_FLAG_READY |FLASH_FLAG_SIZERR |
				FLASH_FLAG_OPTVERR| FLASH_FLAG_OPTVERRUSR);
        while(FLASH_GetStatus()==FLASH_BUSY);
        tmp=*(__IO uint32_t*)Address;
				/*�豸����*/
        DATA_EEPROM_Lock();
    }
    return tmp;
}
