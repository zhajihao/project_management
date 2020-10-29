#include "delay.h"
#include "sys.h"


static float  fac_us=0;//us��ʱ������			   
static u16 fac_ms=0;//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��


//void SysTick_Handler(void)
//{				   
//	
//}

			   
/*��ʼ���ӳٺ���
	��ʹ��ucos��ʱ��,�˺������ʼ��ucos��ʱ�ӽ���
	SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
	SYSCLK:ϵͳʱ��
	0x00��HSI ��Ϊϵͳʱ�� 
	0x04��HSE��Ϊϵͳʱ�� 
	0x08��PLL��Ϊϵͳʱ��*/
void delay_init(u8 SYSCLK)
{ 
 	SysTick->CTRL&=~(1<<3);	/*SYSTICKʹ��PLLʱ��Դ*/	
	fac_us=SYSCLK/8/7.63*1000;				/*�����Ƿ�ʹ��ucos,fac_us����Ҫʹ��*/	 
	fac_ms=(u16)fac_us;/*����ÿ��ms��Ҫ��systickʱ���� */  

}								    
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
/*MSIʱ��Դ�£���ʱ��������10us����100ms��ʱ�����Ϊ1ms��*/
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
} 

			 



































