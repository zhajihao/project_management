#include "spi.h"

//void SPI1_Port_Init(void)
//{
//   
//    
//}
//void SPI1_Init(void)
//{	 		 
//	
//	//����ֻ���SPI�ڳ�ʼ��
//	SPI1_Port_Init();	


//    RCC->AHBENR |=  1<<0;    //ʹ��PORTAʱ��	
//    RCC->APB2ENR |= 1<<12;   	//SPI1ʱ��ʹ�� 
//    
//    RCC->APB2RSTR |= 1<<12;
//    RCC->APB2RSTR &= ~(1<<12);
//    
//    GPIOA->MODER   |= 2<<(5*2);//����
//    GPIOA->OTYPER  |= 1<<5;//�������
//    GPIOA->OSPEEDR |= 1<<(5*2);//2Mhzʱ��
//    GPIOA->PUPDR   |= 1<<(5*2);//����
//    
//    GPIOA->MODER   |= 2<<(6*2);
//    GPIOA->OTYPER  |= 1<<6;
//    GPIOA->OSPEEDR |= 1<<(6*2);
//    GPIOA->PUPDR   |= 1<<(6*2);
//    
//    GPIOA->MODER   |= 2<<(7*2);
//    GPIOA->OTYPER  |= 1<<7;
//    GPIOA->OSPEEDR |= 1<<(7*2);
//    GPIOA->PUPDR   |= 1<<(7*2);
//    
//    GPIOA->AFR[1] &= ~((15<<4*7)|(15<<4*5)|(15<<4*6));
//    GPIOA->AFR[1] |= ((5<<4*7)|(5<<4*5)|(5<<4*6));//�ܽ�ӳ�䵽spi1
//   
//	SPI1->CR1 |= 0<<10;		//ȫ˫��ģʽ	
//    
//	SPI1->CR1 |= 1<<9; 		//���nss����
//	SPI1->CR1 |= 1<<8;  

//	SPI1->CR1 |= 1<<2; 		//SPI����
//	SPI1->CR1 |= 0<<11;		//8bit���ݸ�ʽ	
//	SPI1->CR1 |= 1<<1; 		//����ģʽ��SCKΪ1 CPOL=1
//	SPI1->CR1 |= 1<<0; 		//���ݲ����ӵڶ���ʱ����ؿ�ʼ,CPHA=1  
//	//��SPI2����APB1������.ʱ��Ƶ�����Ϊ36M.
//	SPI1->CR1 |= 3<<3; 		//Fsck=Fpclk1/256
//	SPI1->CR1 |= 0<<7; 		//MSBfirst   
//	SPI1->CR1 |= 1<<6; 		//SPI�豸ʹ��
//	SPI1_ReadWrite(0xff);//��������		 
//}  
void SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
 

//  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;//ʱ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);  
    
 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//����
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;//ʱ������
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);  



    
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);//GPIO_PinSource7
  /* SPI configuration -------------------------------------------------------*/
//  SPI_I2S_DeInit(SPI1);
   /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��spi
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
//  SPI1_SetSpeed(1);
//  SPI1_ReadWrite(0xff);//��������	
}
//SPI2�ٶ����ú���
//SpeedSet:0~7
//SPI�ٶ�=fAPB1/2^(SpeedSet+1)
//APB1ʱ��һ��Ϊ36Mhz
void SPI1_SetSpeed(u8 SpeedSet)
{
	SpeedSet&=0X07;			//���Ʒ�Χ
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SpeedSet<<3;	//����SPI2�ٶ�  
	SPI1->CR1|=1<<6; 		//SPI�豸ʹ��	  
} 
//SPI2 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWrite(u8 TxData)
{		
	u16 retry=0;				 
	while((SPI1->SR&1<<1)==0)		//�ȴ���������	
	{
		retry++;
		if(retry>=0XFFFE)return 0; 	//��ʱ�˳�
	}			  
	SPI1->DR=TxData;	 	  		//����һ��byte 
	retry=0;
	while((SPI1->SR&1<<0)==0) 		//�ȴ�������һ��byte  
	{
		retry++;
		if(retry>=0XFFFE)return 0;	//��ʱ�˳�
	}	  						    
	return SPI1->DR;          		//�����յ�������				    
}









