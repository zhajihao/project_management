/**********************************************************************
 * @File        : analysis.c
 * @Brief  	    : 
 * @ChangeLogs  :
 * Date           Author       Notes
 * 2020-8-12       xiao          the first version
 **********************************************************************/
#include "analysis.h" 
uint8_t TxBuffer[MAX_SIZE] = {0};
uint8_t RxBuffer[MAX_SIZE] = {0};
uint8_t RxFlag = 0; 								 /*0:standy 1:receive ok*/
uint8_t NbrOfDataToTransfer = 0;
uint8_t NbrOfDataToRead = 0;
__IO uint8_t TxCounter = 0;
__IO uint16_t RxCounter = 0;
uint8_t washer_work_flag = 0;
wshaer_status my_washer = {0};
uint8_t app_choose_washer_mode = 0;  /*appѡ���ϴ��ģʽ 0x00��δ֪ģʽ 0x01��ģʽ1 0x02��ģʽ2*/
app_pay_status my_pay = {0};
uint8_t error_flag = 0;       			 /*ϴ���г����쳣�����ǣ�0x00����������Ҫ���δ��룬0x01���쳣�����쳣����ʱ��Ҫ���δ���*/
uint32_t overtime = 0;							 /*��ʱ��������*/
uint8_t overtime_flag = 0;					 /*��ʱ��������ʼ��ֹͣ��־λ*/
/************************************************************************
 * @brief  	ϴ��ģʽѡ�����50ms�������ϴ�»���
 * @param  	uint8_t mode��ѡ���ģʽ��Ŀǰֻ��ģʽ1��ģʽ2���֣�1��ģʽ1
 *          2��ģʽ2
 * @retval 	None	
 * 
 ************************************************************************/
void mode_choose_output(uint8_t mode)
{
	if(mode != 0 && mode != 1)
		return;
	if(mode == 0){
		
			SIGNAL1_OUT(1);
			SIGNAL1_OUT(0);
			delay_ms(TIME_50);
			SIGNAL1_OUT(1);       
	}
	else if(mode == 1)
	{
			SIGNAL2_OUT(1);
			SIGNAL2_OUT(0);
			delay_ms(TIME_50);
			SIGNAL2_OUT(1);
	}
}
/****************************************************************************
 * @brief  	ϴ�»�֧��״̬����������MCU���ݴ������ݺ�ϴ�»�״̬������Ӧ״̬
 * @param  	None
 * @retval 	None	
 * 
 ***************************************************************************/
void washer_work(void)
{
		switch(washer_work_flag)
		{
			case WASHWE_START:
				if(my_washer.think_washer_status == 0x00)
				{
					my_washer.think_washer_status = 0x01;
					mode_choose_output(app_choose_washer_mode);
				}
				if((my_washer.permission_status == ABNORMAL)&& (my_washer.operation_status == ABNORMAL) &&(my_washer.think_washer_status == 0x01))
				{
					washer_work_flag = WASHWE_RUNING;
					my_washer.now_washer_status = 0x01;
				}
				break;	
			case WASHWE_RUNING:
				if((my_washer.permission_status == NORMAL)&&(my_washer.operation_status == ABNORMAL)) /*�������쳣*/
				{
					 error_flag  = 0x01;  /*�쳣��״̬*/
					 eeprom_data[0] = 0x01;
					 eeprom_write_word(eeprom_data,1);
					 my_washer.now_washer_status = 0x02;
					 washer_work_flag = WASHWE_ERROR;
				}
				break;
			case WASHWE_END:
				if(my_washer.think_washer_status == 0x02 && my_pay.app_connection_status == 0x01)
				{
					my_data_pack(0xA6);
				}
				my_washer.think_washer_status = 0x00;
			  washer_work_flag = WASHWE_STANDBY;
				break;
			case WASHWE_STANDBY:
//				if(overtime >= 600)       /*��ʱʱ��Ϊ1min*/
//				{
//					 overtime_flag = 0;
//					 overtime = 0;
//					 if(my_pay.app_connection_status == 0x01)
//					 {
//					  my_data_pack(0xA8); 
//					 }
//				}
//				if(my_washer.now_washer_status == 0x02)
//				{
//					washer_work_flag = WASHWE_ERROR;
//				}
				my_washer.think_washer_status = 0x00;
				app_choose_washer_mode = 3;
				break;
			case WASHWE_ERROR:
				if(my_pay.app_connection_status == 0x01)
				{
					my_data_pack(0xA3);
					delay_ms(500);
				}
				washer_work_flag = WASHWE_STANDBY;
				break;
      default:
				break;	
		}
}
/************************************************************************
 * @brief  	���ظ������������ݽ��д��������
 * @param  	uint8_t Send_ID��Ҫ���͵�ָ��ID
 * @retval 	None	
 * 
 ************************************************************************/
void my_data_pack(uint8_t Send_ID)
{
		
		TxBuffer[0] = 0xAA;
		TxBuffer[1] = 0xAA;
		switch(Send_ID)
		{
			case  0xA1:
				 NbrOfDataToTransfer = 0x06;
				 TxBuffer[4] = 0;
//				 overtime = 0;
//				 overtime_flag = 1;
				 break;
			case  0xA2:
				 NbrOfDataToTransfer = 0x0E;
				 eeprom_read_word(eeprom_data,3);
				 if(eeprom_data[0] == 0x01)
				 {
					 TxBuffer[4] = 0x01;
					 TxBuffer[5] = (uint8_t)(eeprom_data[1] >> 24);
					 TxBuffer[6] = (uint8_t)(eeprom_data[1] >> 16);
					 TxBuffer[7] = (uint8_t)(eeprom_data[1] >> 8);
					 TxBuffer[8] = (uint8_t)(eeprom_data[1]);
					 TxBuffer[9] = (uint8_t)(eeprom_data[2] >> 24);
					 TxBuffer[10] = (uint8_t)(eeprom_data[2] >> 16);
					 TxBuffer[11] = (uint8_t)(eeprom_data[2] >> 8);
					 TxBuffer[12] = (uint8_t)eeprom_data[2];
					 eeprom_data[0] = 0x00;
				   eeprom_write_word(eeprom_data,1);
					 memset(eeprom_data,0,sizeof(eeprom_data));
  				 /*my_pay.indent_status = 0x01;*/
				 }
				 else
				 {
					 TxBuffer[4] = 0x00;
				 }
				 break;
			case  0xA3:
				 NbrOfDataToTransfer = 0x06;
				 if(my_washer.now_washer_status == 0x00)
				 {
					 TxBuffer[4] = 0;
				 }
				 else if(my_washer.now_washer_status == 0x01)
				 {
					 TxBuffer[4] = 1;
				 }
				 else if(my_washer.now_washer_status == 0x03)
				 {
					 TxBuffer[4] = 3;
				 }
				 else
				 {
						TxBuffer[4] = 2;
				 }
//				 if((my_washer.permission_status == NORMAL) && (my_washer.operation_status == NORMAL))
//				 {
//					 TxBuffer[4] = 0;
//				 }
//				 else if(((my_washer.permission_status == ABNORMAL)&&(my_washer.permission_status == ABNORMAL)))
//				 {
//					 TxBuffer[4] = 1;
//				 }
//				 else
//				 {
//						TxBuffer[4] = 2;
//				 }
				 break;
		 case  0xA5:
				 NbrOfDataToTransfer = 0x05;
				 break;
		 case  0xA6:
				 if(error_flag == 0x01)
				 {
					 error_flag = 0x00;
					 TxBuffer[4] = 1;
				 }
				 else
				 {
					 TxBuffer[4] = 0;
				 }
				 NbrOfDataToTransfer = 0x06;
				 break;
			case  0xA8:
				 NbrOfDataToTransfer = 0x05;
				 break;
			default :
				 ;
		}	
		NbrOfDataToTransfer = 0x14;
		TxBuffer[2] = NbrOfDataToTransfer;
		TxBuffer[3] = Send_ID;
		TxBuffer[NbrOfDataToTransfer-1] = my_check_code_calculate(&TxBuffer[3],NbrOfDataToTransfer-4);
		USART_OUT(USART1,TxBuffer,NbrOfDataToTransfer);
		memset(TxBuffer,0,sizeof(TxBuffer));
}
/************************************************************************
 * @brief  	�����������ݣ�������ָ��ID������ش���
 * @param  	uint8_t * data�����������ݰ�����ʼָ��
 * @retval 	None	
 * 
 ************************************************************************/
void my_payload_analysis(uint8_t * data)
{
		uint8_t commandID = data[0];
		switch(commandID)
		{
			case  0xA1:
				 if(memcmp(&data[1], "123456",6)==0)
				 {
						my_data_pack(0xA1);
						my_pay.app_connection_status = 0x01;
				 }
				 break;
			case  0xA2:                      /*APPѯ����һ�����Ƿ�����*/
				 my_data_pack(0xA2); 
				 break;
			case  0xA3:                     /*APPѯ��ϴ�»�״̬*/
				 my_data_pack(0xA3);
				 break;
			case  0xA4: 
				 my_pay.app_connection_status = 0x00;
//				 overtime_flag = 0;
//				 overtime = 0;
				 break;			
			case  0xA5: 
//				 overtime_flag = 0;
//				 overtime = 0;
//				 if(my_washer.now_washer_status == 0x00)
//				 {
						my_data_pack(0xA5);
						washer_work_flag = WASHWE_START;
//				 }
//				 else
////				 {
//					 washer_work_flag = WASHWE_ERROR;
//				 }
				 app_choose_washer_mode = data[1];
				 eeprom_data[0] = 0x00;
				 eeprom_data[1] = (data[2]<<24) | (data[3]<<16) | (data[4]<<8) | (data[5]) ;
				 eeprom_data[2] = (data[6]<<24) | (data[7]<<16) | (data[8]<<8) | (data[9]) ;
				 eeprom_write_word(eeprom_data,3);
				 break;	
			case  0xA6: 
				 /*eeprom_data[0] = 0x00;
				 eeprom_write_word(eeprom_data,1);*/
				 break;	
			default :
				 ;
		}
}
/************************************************************************
 * @brief  	�жϴ������ݵ�֡ͷ��У����Ƿ���ȷ
 *          ���ڽ�����ɱ�־
 * @param  	None
 * @retval  None
 * 
 ************************************************************************/
void my_data_analysis(uint8_t const* r_data, uint16_t len)
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
/************************************************************************
 * @brief  	�жϴ����Ƿ���յ���Ч���ݣ�����Ч�򿽱����������У�����λ��
 *          �ڽ�����ɱ�־
 * @param  	None
 * @retval  None
 * 
 ************************************************************************/
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
/************************************************************************
 * @brief  	����ϵͳʱ�Ӻ�PLLʱ��ԴΪ�ڲ����پ�������������ߵ�Ƶ��
 * @param  	None
 * @retval  None
 * 
 ************************************************************************/
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
/************************************************************************
 * @brief  	���������У�鷽ʽ��У���
 * @param  	uint8_t * data��ָ����������ݻ�������ָ��
 *          uint8_t len�����������ݵĳ���
 * @retval  ���ض�����У���	
 * 
 ************************************************************************/
uint8_t my_check_code_calculate(uint8_t * data, uint8_t len)
{
		uint8_t i,sum = 0;
		for(i=0; i<len; i++)
			sum = (uint8_t)(sum + data[i]);
		sum = 0xff - sum;
		return sum;
}
/************************************************************************
 * @brief  	�ж϶�����У�鷽ʽ��У����Ƿ���ȷ
 * @param  	uint8_t * data��ָ���У�����ݻ�������ָ��
 *          uint8_t len����У�����ݵĳ���
 * @retval 	У��ɹ�����0��ʧ�ܷ���2	
 * 
 ************************************************************************/
uint8_t my_check_code_analysis(uint8_t * data, uint8_t len)
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
/************************************************************************
 * @brief  	�ж����ȼ����ã����ô���1���ж����ȼ�
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/************************************************************************
 * @brief  	��ʼ������1�����ô��ڽ��ա��������ţ����ñ����ʵ�
 * @param  	USART_TypeDef* USARTx�����ں�
 *          uint8_t *Data��ָ���ͻ�������ָ��
 *          uint16_t Len�����͵����ݳ���
 * @retval 	None	
 * 
 ************************************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len)
{ 
	uint16_t i;
	for(i=0; i<Len; i++){
		USART_SendData(USARTx, Data[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
//	USART_SendData(USARTx, '\r');
//	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
//	USART_SendData(USARTx, '\n');
//	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
}
/************************************************************************
* @brief  	��ʼ������1�����ô��ڽ��ա��������ţ����ñ����ʵ�
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void USART_INIT(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/**��ʼ������1ʱ�ӡ�GPIOAʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	
	
	/*��λ����1*/
	USART_DeInit(USART1); 
	
	/*USART1_TX   PA.9*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	/*USART1_RX	  PA.10*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
/************************************************************************
 * @brief  	��ȡCRCУ���һ�ֽ�У�飨��ʽΪ32λ��
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
uint8_t git_crc_num(uint8_t *pbuffer,uint8_t size)
{
	int i = 0;
	CRC_ResetDR();
	for(i=0;i<size;i++)
	{
		CRC_CalcCRC(pbuffer[i]);
	}
	return CRC_GetCRC();
	
}
/************************************************************************
 * @brief  	��ȡϴ�»�״̬����ȡϴ�»�������/�����źţ����/������ź�
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void get_washer_status(void)
{
	/*��ȡ���״̬*/
	if(!GPIO_ReadInputDataBit(WASHER_PERMISSION_INPort,WASHER_PERMISSION_INPin))
	{
		delay_ms(50);
		if(!GPIO_ReadInputDataBit(WASHER_PERMISSION_INPort,WASHER_PERMISSION_INPin))
		{
			my_washer.permission_status = NORMAL;
		}
		else
		{
			my_washer.permission_status = ABNORMAL;
		}
	}
	else
	{
		my_washer.permission_status = ABNORMAL;
	}
	/*��ȡϴ�»�״̬*/
	if(GPIO_ReadInputDataBit(WASHER_STATUS_INPort,WASHER_STATUS_INPin))
	{
		delay_ms(50);
		if(GPIO_ReadInputDataBit(WASHER_STATUS_INPort,WASHER_STATUS_INPin))
		{
			my_washer.operation_status = NORMAL;
		}
		else
		{
			my_washer.operation_status = ABNORMAL;
		}
	}
	else
	{
		my_washer.operation_status = ABNORMAL;
	}
	/*ʵʱ״̬�ж�*/
	if((my_washer.permission_status == NORMAL) && (my_washer.operation_status == NORMAL))
	{                                                          
			my_washer.now_washer_status = 0x00;            /*����״̬*/
	}
	else if((my_washer.permission_status == ABNORMAL) && (my_washer.operation_status == NORMAL) && (my_washer.think_washer_status != 0x01)) /*����״̬�µ��쳣*/
	{
		my_washer.now_washer_status = 0x03;                            /*�ű�����*/
	}
	else if((my_washer.permission_status == ABNORMAL) && (my_washer.operation_status == ABNORMAL))
	{
		my_washer.now_washer_status = 0x01;                            /*����������״̬*/
	}
	
	if((my_washer.operation_status == NORMAL)&&(my_washer.think_washer_status == 0x01)&&(washer_work_flag == WASHWE_RUNING)) /*��ϴ��״̬�ص�����״̬*/
	{
				my_washer.think_washer_status = 0x02;                       /*ϴ�����*/
				washer_work_flag = WASHWE_END;
	}
}