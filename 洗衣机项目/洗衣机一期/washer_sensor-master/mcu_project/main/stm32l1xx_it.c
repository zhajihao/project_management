/**
  ******************************************************************************
  * @file    TIM/TimeBase/stm32l1xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-December-2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "general_type.h"
#include "stm32l1xx_usart.h"
#include "string.h"
#include "stm32l1xx_exti.h"
#include "led.h"
#include "stm32l1xx_tim.h"
u8 flag;
/** @addtogroup STM32L1xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_TimeBase
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t capture = 0;
extern __IO uint16_t CCR1_Val;
extern __IO uint16_t CCR2_Val;
extern __IO uint16_t CCR3_Val;
extern __IO uint16_t CCR4_Val;


//#define USART_TypeDef  USART1
#define USARTx_IRQHANDLER   USART1_IRQHandler

#define MAX_SIZE 200
#define countof(a)   (sizeof(a) / sizeof(*(a)))

uint8_t TxBuffer[MAX_SIZE];
uint8_t RxBuffer[MAX_SIZE];
uint8_t RxFlag = 0; // 0:standy 1:receive ok
uint8_t NbrOfDataToTransfer = 0;
uint8_t NbrOfDataToRead = 0;
__IO uint8_t TxCounter = 0;
__IO uint16_t RxCounter = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles USARTx global interrupt request.
  * @param  None
  * @retval None
  */
enum {
	RCV_STANDBY = 0,
	RCV_STATE_HEAD,
	RCV_STATE_SIZE,
	RCV_STATE_DATA,
	RCV_END,
};

void reveice_data(uint8_t data)
{
	static uint8_t rcv_flag = RCV_STANDBY;
	switch(rcv_flag)
	{
		case RCV_STANDBY: 
			if(data == 0xAA) {
				RxFlag = 0;
				RxCounter = 0;
				NbrOfDataToRead = 0;
				memset(RxBuffer,0,MAX_SIZE);				
				rcv_flag = RCV_STATE_HEAD;
				RxBuffer[RxCounter++] = data;
			};
			break;
		case RCV_STATE_HEAD:
			if(data == 0xAA) {
				rcv_flag = RCV_STATE_SIZE;
				RxBuffer[RxCounter++] = data;
			} else {
				rcv_flag = RCV_STANDBY;
			}
			break;
		case RCV_STATE_SIZE:
			if(data > MAX_SIZE || data < 4) {
				rcv_flag = RCV_STANDBY;
				break;
			}
			NbrOfDataToRead = data;
			RxBuffer[RxCounter++] = data;
			rcv_flag = RCV_STATE_DATA;
			break;			
		case RCV_STATE_DATA:
			RxBuffer[RxCounter++] = data;
			if(NbrOfDataToRead <= RxCounter){
				rcv_flag = RCV_END;				
			} else {
				break;
			}
		case RCV_END: 
			rcv_flag = RCV_STANDBY;
			RxFlag = 1;
			break;
		default:
			rcv_flag = RCV_STANDBY;
			break;
	}
}

void USARTx_IRQHANDLER(void)
{
	uint8_t ret = 0;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    ret = USART_ReceiveData(USART1);
		//USART_SendData(USART_TypeDef, ret);
		reveice_data(ret);
  }
}


void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		if(Money_Judge.Faling_Flag1 == 0x01)
			{
				Money_Judge.Faling_Flag1 = 0x00;
				Money_Judge.Count_Flag[0] = 1;
			}
	}
	EXTI_ClearITPendingBit(EXTI_Line10);
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)       //100 out先输出大脉宽
	{
				Money_Judge.Faling_Flag1 = 0x01;
				Money_Judge.Count_Flag[1] = 1;
	}
	EXTI_ClearITPendingBit(EXTI_Line11);
		if(EXTI_GetITStatus(EXTI_Line12) != RESET)      //50 out先输出大脉宽
	{
				Money_Judge.Faling_Flag2 = 0x01;
				Money_Judge.Count_Flag[2] = 1;
	}
	EXTI_ClearITPendingBit(EXTI_Line12);
		if(EXTI_GetITStatus(EXTI_Line13) != RESET)    
	{
			if(Money_Judge.Faling_Flag2 == 0x01)
			{
				Money_Judge.Faling_Flag2 = 0x00;
				Money_Judge.Count_Flag[3] = 1;
			}
	}
	EXTI_ClearITPendingBit(EXTI_Line13);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		if(Money_Judge.Count_Flag[0] == 1)
		{
			if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10))
			{
				Money_Judge.Count_Num[0]++;
			}
			else
			{
				Money_Judge.Count_Flag[0] = 2;
			}
		}
		if(Money_Judge.Count_Flag[1] == 1)
		{
			if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11))
			{
				Money_Judge.Count_Num[1]++;
			}
			else
			{
				Money_Judge.Count_Flag[1] = 2;
			}
		}
		if(Money_Judge.Count_Flag[2] == 1)
		{
			if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12))
			{
				Money_Judge.Count_Num[2]++;
			}
			else
			{
				Money_Judge.Count_Flag[2] = 2;
			}
		}
		if(Money_Judge.Count_Flag[3] == 1)
		{
			if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13))
			{
				Money_Judge.Count_Num[3]++;
			}
			else
			{
				Money_Judge.Count_Flag[3] = 2;
			}
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{}

/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
