#include "led.h"
#include "general_type.h"
#include "delay.h"
#include "stm32l1xx_exti.h"
#include "stm32l1xx_syscfg.h" 
#include "stm32l1xx_tim.h"
#include "stm32l1xx_rtc.h"
#include "stm32l1xx_pwr.h"
#include "stm32l1xx_usart.h"
#include "string.h"
void washer_sold_coins(uint8_t num, uint8_t state);
Money_Judge_Typedef Money_Judge ={0};
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
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_14;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_14); 
  GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);  
  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource10);    //选择输入线
	EXTI_InitStruct.EXTI_Line = EXTI_Line10; 							//选择EXTI线
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;						//选择模式
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;		//选择下降沿触发
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;				//使能选择的EXTI_Line
	EXTI_Init(&EXTI_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource11);    //选择输入线
	EXTI_InitStruct.EXTI_Line = EXTI_Line11; 							//选择EXTI线
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;						//选择模式
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;		//选择下降沿触发
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;				//使能选择的EXTI_Line
	EXTI_Init(&EXTI_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource12);    //选择输入线
	EXTI_InitStruct.EXTI_Line = EXTI_Line12; 							//选择EXTI线
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;						//选择模式
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;		//选择下降沿触发
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;				//使能选择的EXTI_Line
	EXTI_Init(&EXTI_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource13);    //选择输入线
	EXTI_InitStruct.EXTI_Line = EXTI_Line13; 							//选择EXTI线
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;						//选择模式
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;		//选择下降沿触发
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;				//使能选择的EXTI_Line
	EXTI_Init(&EXTI_InitStruct);
}

void MY_GPIO_TEST(void)
{
	RELAY_ON(1);
	SIGNAL1_OUT(1);
	SIGNAL2_OUT(1);
	SIGNAL3_OUT(1);
	SIGNAL4_OUT(1);
	delay_ms(2000);
	RELAY_ON(0);
	SIGNAL1_OUT(0);
	SIGNAL2_OUT(0);
	SIGNAL3_OUT(0);
	SIGNAL4_OUT(0);
	delay_ms(2000);
}
/*MSI时钟源下，定时器误差在5ms左右*/
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

	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM2,ENABLE);//timer enable
}

void Judge_Money(void)
{
	if(Money_Judge.Count_Flag[0] == 2)
	{
		Money_Judge.Count_Flag[0] = 0;
			if((Money_Judge.Count_Num[1] >= 0) && (Money_Judge.Count_Flag[1]== 2))
			{
				Money_Judge.Count_Flag[1] = 0;
				Money_Judge.Count_Num[1] = 0;
				if( Money_Judge.Count_Num[0] >= 0)
				{
					Money_Judge.Count_Num[0] = 0;
					washer_sold_coins(1,0);
				}
			}
	}
	if(Money_Judge.Count_Flag[3] == 2)
	{
		Money_Judge.Count_Flag[3] = 0;
		if((Money_Judge.Count_Num[2] >= 0) && (Money_Judge.Count_Flag[2]== 2))         
		{
			Money_Judge.Count_Num[2] = 0;
			Money_Judge.Count_Flag[2] = 0;
			if(Money_Judge.Count_Num[3] >= 0)
			{
				Money_Judge.Count_Num[3] = 0;
				washer_sold_coins(1,1);
			}
		}
	}
}

void PowerInit(void)
{
	RTC_InitTypeDef  RTC_InitStruct;
	RTC_TimeTypeDef  RTC_TimeStruct;
	
	RCC_MSIRangeConfig(RCC_MSIRange_4);
	RCC_MSICmd(ENABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_MSI);
//	RCC_LSICmd(ENABLE);
//	RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div16);
//	
//	RTC_EnterInitMode();
//	
//	RTC_InitStruct.RTC_AsynchPrediv = (100-1);
//	RTC_InitStruct.RTC_HourFormat = 
//	RTC_InitStruct.RTC_SynchPrediv = 
//	RTC_Init(&RTC_InitStruct);
//	
//	RTC_ExitInitMode();
	
	
	
	PWR_EnterLowPowerRunMode(ENABLE);

		
}
void my_delay_ms(u16 num)
{

  u16 Counter,i;
	for(i = 0; i < num; i++)
	{
		for(Counter = 0; Counter < 100; Counter++);
	}
	
}

void SysTickConfig(void)
{
  /* SysTick interrupt each 250 ms with SysTick Clock equal to 4MHz */
//  if (SysTick_Config((SystemCoreClock/8) / 4))
//  { 
	 if (SysTick_Config(SystemCoreClock))
  { 
    /* Capture error */ 
    while (1);
  }

  /* Select AHB clock(HCLK) divided by 8 as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

  /* Set SysTick Preemption Priority to 1 */
  NVIC_SetPriority(SysTick_IRQn, 0x04);
}
