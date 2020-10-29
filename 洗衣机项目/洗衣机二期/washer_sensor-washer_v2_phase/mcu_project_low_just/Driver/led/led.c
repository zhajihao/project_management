/**********************************************************************
 * @File        : led.c
 * @Brief  	    : 
 * @ChangeLogs  :
 * Date           Author       Notes
 * 2020-8-12       xiao          the first version
 **********************************************************************/
#include "led.h"
#include "general_type.h"
#include "delay.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_syscfg.h" 
#include "stm32l1xx_tim.h"
#include "string.h"
void washer_sold_coins(uint8_t num, uint8_t state);
Money_Judge_Typedef Money_Judge ={0};
/************************************************************************
 * @brief  	初始化GPIO，配置各个GPIO的模式、速率等
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void MY_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStruct;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 		//开时钟
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15 |GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
  GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_14 | GPIO_Pin_15);   
  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //2、3
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource2);    //选择输入线
//	EXTI_InitStruct.EXTI_Line = EXTI_Line2; 							//选择EXTI线
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;						//选择模式
//	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;		//选择下降沿触发
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;				//使能选择的EXTI_Line
//	EXTI_Init(&EXTI_InitStruct);

//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource3);    //选择输入线
//	EXTI_InitStruct.EXTI_Line = EXTI_Line3; 							//选择EXTI线
//	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;						//选择模式
//	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;		//选择下降沿触发
//	EXTI_InitStruct.EXTI_LineCmd = ENABLE;				//使能选择的EXTI_Line
//	EXTI_Init(&EXTI_InitStruct);
}
/************************************************************************
 * @brief  	测试GPIO
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void MY_GPIO_TEST(void)
{
//	RELAY_ON(1);
	SIGNAL1_OUT(1);
	SIGNAL2_OUT(1);
//	SIGNAL3_OUT(1);
//	SIGNAL4_OUT(1);
	delay_ms(2000);
//	RELAY_ON(0);
	SIGNAL1_OUT(0);
	SIGNAL2_OUT(0);
//	SIGNAL3_OUT(0);
//	SIGNAL4_OUT(0);
	delay_ms(2000);
}
/************************************************************************
 * @brief  	定时器二初始化，初始化分频系数、技术个数、技术模式等等
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void Timer2_Init(uint16_t Period,uint16_t Prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	TIM_DeInit(TIM2);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	TIM_TimeBaseInitStruct.TIM_Period = Period-1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = Prescaler-1;
	TIM_TimeBaseInitStruct.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	//TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);

	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2,DISABLE);//timer enable
}
/************************************************************************
 * @brief  	控制PB9脚，实现LED灯的闪烁
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void test_led(void)
{	
	GPIO_SetBits(GPIOB, GPIO_Pin_9);
	delay_ms(500);
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);
	delay_ms(500);
}
/************************************************************************
 * @brief  	初始化打印调试串口，配置波特率等
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void USART3_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	/*初始化GPIO口时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	
	
	/*复位串口1*/
	USART_DeInit(USART3); 
	
	/*配置串口3发送引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	/*配置串口3接收引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*串口复用*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	/*配置波特率为9600,8位数据位，1位停止位、无校验等*/
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3,&USART_InitStructure);
	USART_Cmd(USART3,ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}
