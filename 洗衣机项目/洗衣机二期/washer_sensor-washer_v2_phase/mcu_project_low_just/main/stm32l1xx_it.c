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
#include "analysis.h"
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

#define USARTx_IRQHANDLER   USART1_IRQHandler

#define countof(a)   (sizeof(a) / sizeof(*(a)))

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
/************************************************************************
 * @brief  	串口1数据解析，判断帧头、长度等是否正确
 * @param  	uint8_t data：串口接收到的数据
 * @retval 	None	
 * 
 ************************************************************************/
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
/************************************************************************
 * @brief  	串口1接受中断函数
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void USARTx_IRQHANDLER(void)
{
	uint8_t ret = 0;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    ret = USART_ReceiveData(USART1);
		//USART_SendData(USART3, ret);
		//USART_SendData(USART_TypeDef, ret);
		reveice_data(ret);
  }
}
/************************************************************************
 * @brief  	串口3接受中断函数
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void USART3_IRQHandler(void)
{
	uint8_t ret = 0;
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    ret = USART_ReceiveData(USART3);
		/*USART test**************************/
		/*USART_SendData(USART3, ret);********/
		/*Usart3_reveice_data(ret);*/
  }
}
/************************************************************************
 * @brief  	定时器二中断处理函数
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		if(overtime_flag == 1)
		{
			overtime++;
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
