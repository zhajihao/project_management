/**********************************************************************
 * @File        : analysis.h
 * @Brief  	    : 
 * @ChangeLogs  :
 * Date           Author       Notes
 * 2020-8-12       xiao          the first version
 **********************************************************************/
#ifndef ANALYSIS_H
#define ANALYSIS_H
#include "stm32l1xx.h"
#include "stm32l1xx_usart.h"
#include "stm32l1xx_gpio.h"
#include "sys.h"
#include "misc.h"
#include "delay.h"
#include "led.h"
#include "string.h"
#include "stm32l1xx_crc.h"
#include "eeprom.h"
#define TIME_X 100  // 25ms - 150ms
#define TIME_Y 120  // 40ms - 200ms
#define TIME_Z 50  // 5ms - 100ms

#define TIME_50 50  /*50ms������*/
#define MAX_SIZE 200
typedef struct 
{
	uint8_t operation_status;  /*0x00��δ֪״̬ 0x01������״̬ 0x02��������״̬*/
	uint8_t permission_status; /*0x00��δ֪״̬ 0x01����� 0x02�������*/
	uint8_t now_washer_status; /*ʵʱ��ȡ��ϴ�»���״̬0x00������״̬ 0x01��ϴ����  0x02�������쳣*/
	uint8_t think_washer_status; /*�����ϴ�»�״̬ 0x00������״̬ 0x01��ϴ���� 0x02��ϴ����� 0x03�������쳣*/
}wshaer_status;

typedef struct 
{
	uint8_t indent_status;          /*0x00������״̬����һ�������쳣�� 0x01���쳣״̬����һ���������践�����ţ�*/
	uint8_t app_connection_status;  /*0x00��appδ����״̬ 0x01��app������״̬*/
}app_pay_status;

extern uint8_t TxBuffer[];
extern uint8_t RxBuffer[];
extern uint8_t RxFlag; // 0:standy 1:receive ok
extern uint8_t NbrOfDataToTransfer;
extern uint8_t NbrOfDataToRead;
extern __IO uint8_t TxCounter; 
extern __IO uint16_t RxCounter; 

extern uint8_t washer_work_flag;
extern wshaer_status my_washer;
extern uint8_t app_choose_washer_mode;
extern app_pay_status my_pay;
extern uint8_t error_flag; 
extern uint32_t overtime;										
extern uint8_t overtime_flag;								
enum
{
		WASHWE_STANDBY = 0, 
    WASHWE_START,
    WASHWE_END,
    WASHWE_RUNING,
		WASHWE_ERROR,
};
enum
{
    NORMAL = 1,
    ABNORMAL = 2,
};
void washer_work(void);
void USART_INIT(void);
void NVIC_Config(void);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len);
uint8_t my_check_code_analysis(uint8_t * data, uint8_t len);
uint8_t my_check_code_calculate(uint8_t * data, uint8_t len);
void analysis_data(void);
void system_clk_set(void);
void my_data_analysis(uint8_t const* r_data, uint16_t len);
void my_payload_analysis(uint8_t * data);
void my_data_pack(uint8_t Send_ID);
uint8_t git_crc_num(uint8_t *pbuffer,uint8_t size);
void mode_choose_output(uint8_t mode);
void get_washer_status(void);
#endif //ANALYSIS_H

