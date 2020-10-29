/**********************************************************************
 * @File        : main.c
 * @Brief  	    : 
 * @ChangeLogs  :
 * Date           Author       Notes
 * 2020-8-12       xiao          the first version
 **********************************************************************/
#include "analysis.h"
#include "eeprom.h"
#define MODE_DEBUG 0
int main(void)
{
		/*定义时钟类型变量*/
		RCC_ClocksTypeDef RCC_Clocks_m;
	
		/*初始化中断优先级*/
		NVIC_Config();
	
		/*串口一初始化*/
		USART_INIT();
		
		/*打印调试串口初始化*/
		USART3_INIT();
	
		/*初始化滴答定时器*/
		delay_init(32);
	
		/*初始化GPIO*/
		MY_GPIO_Init();
	
		/*查看时钟频率*/
		RCC_GetClocksFreq(&RCC_Clocks_m);
	
		/*开启100ms的定时器*/
		Timer2_Init(1000,3200);
		
		/*MCU默认为待机模式*/
		washer_work_flag = WASHWE_STANDBY;
    while(1)
    {	
//			/*测试用*/
			GPIO_SetBits(GPIOB, GPIO_Pin_2);
			GPIO_SetBits(GPIOB, GPIO_Pin_3);
			delay_ms(50);
			GPIO_ResetBits(GPIOB, GPIO_Pin_2);
			GPIO_ResetBits(GPIOB, GPIO_Pin_3);
			delay_ms(10);
//			/*蓝牙串口数据解析*/
//			analysis_data();
//			
//			/*实时获取洗衣机状态*/
//			get_washer_status();
//			
//			/*洗衣支付状态机*/
//			washer_work();
//			#if MODE_DEBUG
//			MY_GPIO_TEST();
//			#endif
		} 
}
 



