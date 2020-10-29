#include "stm32l1xx.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_gpio.h"
#include "sys.h"
#include "misc.h"
#include "delay.h"
#include "led.h"
#include "string.h"

#define TIME_X 100  // 25ms - 150ms
#define TIME_Y 120  // 40ms - 200ms
#define TIME_Z 50  // 5ms - 100ms
#define MODE_DEBUG 0
void USART_INIT(void);
void NVIC_Config(void);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len);
static uint8_t my_check_code_analysis(uint8_t * data, uint8_t len);
static uint8_t my_check_code_calculate(uint8_t * data, uint8_t len);
void washer_sold_coins(uint8_t num, uint8_t state);
void washer_work(void);
void analysis_data(void);

USART_InitTypeDef USART_InitStructure;
extern uint8_t TxBuffer[];
extern uint8_t RxBuffer[];
extern uint8_t RxFlag; // 0:standy 1:receive ok
extern uint8_t NbrOfDataToTransfer;
extern uint8_t NbrOfDataToRead;
extern __IO uint8_t TxCounter; 
extern __IO uint16_t RxCounter; 

static uint8_t washer_work_flag;
static uint8_t m_sold_coins_100,m_sold_coins_50;

enum
{
    WASHWE_START = 0,
    WASHWE_END,
    WASHWE_SOLD,
		WASHWE_STANDBY,
};

#define MSI_NO
int main(void)
{
		RCC_ClocksTypeDef RCC_Clocks_m;
#ifdef MSI_NO
		PowerInit();
#endif
		NVIC_Config();
		USART_INIT();
		delay_init(8);
		MY_GPIO_Init();
		RCC_GetClocksFreq(&RCC_Clocks_m);
		washer_work_flag = WASHWE_STANDBY;
		Timer2_Init(10,104);             //1ms
    while(1)
    {
			Judge_Money();
			analysis_data();
			washer_work();
			#if MODE_DEBUG
			MY_GPIO_TEST();
			#endif
		} 
}


void washer_start(void)
{
	//RELAY_ON(1);
}

void washer_end(void)
{
	//RELAY_ON(0);
}

//state: 100=0; 50=1
void washer_sold_coins(uint8_t num, uint8_t state)
{
	if(num==0 || state>1)
		return;
	if(state == 0){
		while(num--){
			SIGNAL1_OUT(1);
			SIGNAL2_OUT(1);

			SIGNAL1_OUT(0);
			delay_ms(TIME_X);
			SIGNAL1_OUT(1);
			delay_ms(TIME_Y-TIME_X);
			SIGNAL2_OUT(0);
			delay_ms(TIME_Z);
			SIGNAL2_OUT(1);
			delay_ms(500);        //2020.7.16测试用
		}
	}
	else
	{
		while(num--){
			SIGNAL3_OUT(1);
			SIGNAL4_OUT(1);

			SIGNAL3_OUT(0);
			delay_ms(TIME_X);
			SIGNAL3_OUT(1);
			delay_ms(TIME_Y-TIME_X);
			SIGNAL4_OUT(0);
			delay_ms(TIME_Z);
			SIGNAL4_OUT(1);
			delay_ms(500);      //2020.7.16测试用
		}
	}


}

void washer_sold(uint8_t num_100,uint8_t num_50)
{
	if(num_100==0 && num_50==0)
		return;
	washer_sold_coins(num_100,0);
	washer_sold_coins(num_50,1);
}

void washer_work(void)
{
		switch(washer_work_flag)
		{
			case WASHWE_START:
				washer_start();
			  washer_work_flag = WASHWE_STANDBY;
				break;	
			case WASHWE_END:
				washer_end();
			  washer_work_flag = WASHWE_STANDBY;
				break;
			case WASHWE_SOLD:
				washer_sold(m_sold_coins_100,m_sold_coins_50);
				washer_work_flag = WASHWE_STANDBY;
			case WASHWE_STANDBY:
				m_sold_coins_100 = 0;
				m_sold_coins_50 = 0;
				break;
      default:
				break;	
		}
}

static void my_data_pack(uint8_t Send_ID)
{
		
		TxBuffer[0] = 0xAA;
		TxBuffer[1] = 0xAA;
		switch(Send_ID)
		{
			case  0xA1:
				 NbrOfDataToTransfer = 0x06;
				 TxBuffer[4] = 0;
				 break;
			case  0xA2:
				 NbrOfDataToTransfer = 0x06;
				 TxBuffer[4] = 0;
				 break;
			case  0xA3:
				 NbrOfDataToTransfer = 0x06;
				 TxBuffer[4] = 0;
				 break;
			default :
				 ;
		}	
		TxBuffer[2] = NbrOfDataToTransfer;
		TxBuffer[3] = Send_ID;
		TxBuffer[NbrOfDataToTransfer-1] = my_check_code_calculate(&TxBuffer[3],NbrOfDataToTransfer-4);
		USART_OUT(USART1,TxBuffer,NbrOfDataToTransfer);
}

static void my_payload_analysis(uint8_t * data)
{
		uint8_t commandID = data[0];
		switch(commandID)
		{
			case  0xA1:
				 if(memcmp(&data[1], "123456",6)==0)
				 {				
						my_data_pack(0xA1);
					  washer_work_flag = WASHWE_START;
				 }
				 break;
			case  0xA2:
				 my_data_pack(0xA2);
				 m_sold_coins_100 = data[1];
				 m_sold_coins_50 = 0;
				 washer_work_flag = WASHWE_SOLD;
				 break;
			case  0xA3:
				 my_data_pack(0xA3);
				 m_sold_coins_100 = data[1];
				 m_sold_coins_50 = data[2];
			   washer_work_flag = WASHWE_SOLD;
				 break;
			case  0xA4:
			   washer_work_flag = WASHWE_END;
				 break;			
			default :
				 ;
		}
}

static void my_data_analysis(uint8_t const* r_data, uint16_t len)
{
	uint8_t error = 0;
	uint8_t data[200];
	uint8_t size = len;

	if(r_data == NULL || len == 0)
		return;
	memcpy(data,r_data,size);
	if(data[0] == 0xAA && data[1] == 0xAA)
	{
		uint8_t my_len = data[2];
		uint8_t p_len = my_len-3;
		error = my_check_code_analysis(&data[3],p_len);
		if(error == 0)
		{
			my_payload_analysis(&data[3]);
		}
	}
}

void analysis_data(void)
{
	uint8_t size = NbrOfDataToRead;
	uint8_t data[200];

	if(RxFlag != 1 || size == 0)
		return;
	RxFlag = 0;

	memcpy(data,RxBuffer,size);
	NbrOfDataToTransfer = size;
	#if MODE_DEBUG
	USART_OUT(USART1,data,size);
	#endif
	my_data_analysis(data,size);
}
void system_clk_set(void)
{
	RCC_DeInit();
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_PLLConfig(RCC_PLLSource_HSI,RCC_PLLMul_6,RCC_PLLDiv_3);
	RCC_HSICmd(ENABLE);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while(RCC_GetSYSCLKSource() != 0x0C);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
}
static uint8_t my_check_code_calculate(uint8_t * data, uint8_t len)
{
		uint8_t i,sum = 0;
		for(i=0; i<len; i++)
			sum = (uint8_t)(sum + data[i]);
		sum = 0xff - sum;
		return sum;
}

static uint8_t my_check_code_analysis(uint8_t * data, uint8_t len)
{
		uint8_t i,sum = 0;
		for(i=0; i<len; i++)
			sum = (uint8_t)(sum + data[i]);
		if(sum != 0xff)
		{
			return 2;
		}	
		else
		{
			return 0;
		}
}

void NVIC_Config(void)  //串口接收 GPIO 中断初始化
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len)
{ 
	uint16_t i;
	for(i=0; i<Len; i++){
		USART_SendData(USARTx, Data[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
	USART_SendData(USARTx, '\r');
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	USART_SendData(USARTx, '\n');
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
}

void USART_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//使能USART1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	//GPIOA时钟
	
	USART_DeInit(USART1);  //复位串口1
	
 //USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	
	USART_InitStructure.USART_BaudRate = 1200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}


