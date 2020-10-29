#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
#include "general_type.h"
#include "stm32l1xx.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"

typedef struct{
uint8_t Count_Flag[4];  //计时标志
uint16_t Count_Num[4];  //计时数
uint8_t Faling_Flag1;   //通道1开始计时标志 
uint8_t Faling_Flag2;   //通道2开始计时标志
}Money_Judge_Typedef;

extern Money_Judge_Typedef Money_Judge;

void MY_GPIO_Init(void);//初始化

#define RELAY_PORT 14
#define RELAY_ON(sts) (sts?( GPIOB->ODR |= (unsigned int)(1<<RELAY_PORT) ):( GPIOB->ODR &= ~((unsigned int)(1<<RELAY_PORT) )))

#define SIGNAL1_PORT 1
#define SIGNAL1_OUT(sts) (sts?( GPIOB->ODR |= (unsigned int)(1<<SIGNAL1_PORT) ):( GPIOB->ODR &= ~((unsigned int)(1<<SIGNAL1_PORT) )))

#define SIGNAL2_PORT 0
#define SIGNAL2_OUT(sts) (sts?( GPIOB->ODR |= (unsigned int)(1<<SIGNAL2_PORT) ):( GPIOB->ODR &= ~((unsigned int)(1<<SIGNAL2_PORT) )))

#define SIGNAL3_PORT 3
#define SIGNAL3_OUT(sts) (sts?( GPIOB->ODR |= (unsigned int)(1<<SIGNAL3_PORT) ):( GPIOB->ODR &= ~((unsigned int)(1<<SIGNAL3_PORT) )))

#define SIGNAL4_PORT 2
#define SIGNAL4_OUT(sts) (sts?( GPIOB->ODR |= (unsigned int)(1<<SIGNAL4_PORT) ):( GPIOB->ODR &= ~((unsigned int)(1<<SIGNAL4_PORT) )))
void MY_GPIO_TEST(void);
void Timer2_Init(uint16_t Period,uint16_t Prescaler);
void Judge_Money(void);
void PowerInit(void);
void my_delay_ms(u16 num);
void SysTickConfig(void);
#endif

















