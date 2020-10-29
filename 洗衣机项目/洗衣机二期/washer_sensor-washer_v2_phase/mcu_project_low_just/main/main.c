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
		/*����ʱ�����ͱ���*/
		RCC_ClocksTypeDef RCC_Clocks_m;
	
		/*��ʼ���ж����ȼ�*/
		NVIC_Config();
	
		/*����һ��ʼ��*/
		USART_INIT();
		
		/*��ӡ���Դ��ڳ�ʼ��*/
		USART3_INIT();
	
		/*��ʼ���δ�ʱ��*/
		delay_init(32);
	
		/*��ʼ��GPIO*/
		MY_GPIO_Init();
	
		/*�鿴ʱ��Ƶ��*/
		RCC_GetClocksFreq(&RCC_Clocks_m);
	
		/*����100ms�Ķ�ʱ��*/
		Timer2_Init(1000,3200);
		
		/*MCUĬ��Ϊ����ģʽ*/
		washer_work_flag = WASHWE_STANDBY;
    while(1)
    {	
//			/*������*/
			GPIO_SetBits(GPIOB, GPIO_Pin_2);
			GPIO_SetBits(GPIOB, GPIO_Pin_3);
			delay_ms(50);
			GPIO_ResetBits(GPIOB, GPIO_Pin_2);
			GPIO_ResetBits(GPIOB, GPIO_Pin_3);
			delay_ms(10);
//			/*�����������ݽ���*/
//			analysis_data();
//			
//			/*ʵʱ��ȡϴ�»�״̬*/
//			get_washer_status();
//			
//			/*ϴ��֧��״̬��*/
//			washer_work();
//			#if MODE_DEBUG
//			MY_GPIO_TEST();
//			#endif
		} 
}
 



