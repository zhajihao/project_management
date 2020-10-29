#ifndef __SPI_H
#define __SPI_H 			   
#include "stm32l1xx.h" 	
#include "general_type.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_spi.h"
#include "stm32l1xx_rcc.h"
// SPI�����ٶ����� 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		4
#define SPI_SPEED_64 		5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7





void SPI1_Port_Init(void);
void SPI1_Init(void);			 //��ʼ��SPI2��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI2�ٶ�   
u8 SPI1_ReadWrite(u8 TxData);//SPI2���߶�дһ���ֽ�
		 

#endif





























