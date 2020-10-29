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
uint8_t app_choose_washer_mode = 0;  /*app选择的洗衣模式 0x00：未知模式 0x01：模式1 0x02：模式2*/
app_pay_status my_pay = {0};
uint8_t error_flag = 0;       			 /*洗衣中出现异常，需标记，0x00：正常不需要二次存入，0x01：异常，再异常出现时就要二次存入*/
uint32_t overtime = 0;							 /*定时器计数器*/
uint8_t overtime_flag = 0;					 /*定时器计数开始和停止标志位*/
/************************************************************************
 * @brief  	洗衣模式选择（输出50ms的脉宽给洗衣机）
 * @param  	uint8_t mode：选择的模式（目前只有模式1和模式2两种）1：模式1
 *          2：模式2
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
 * @brief  	洗衣机支付状态机处理函数，MCU根据串口数据和洗衣机状态进入相应状态
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
				if((my_washer.permission_status == NORMAL)&&(my_washer.operation_status == ABNORMAL)) /*运行中异常*/
				{
					 error_flag  = 0x01;  /*异常中状态*/
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
//				if(overtime >= 600)       /*超时时间为1min*/
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
 * @brief  	将回复给蓝牙的数据进行打包并发送
 * @param  	uint8_t Send_ID：要发送的指令ID
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
 * @brief  	解析蓝牙数据，并根据指令ID进行相关处理
 * @param  	uint8_t * data：待解析数据包的起始指针
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
			case  0xA2:                      /*APP询问上一订单是否正常*/
				 my_data_pack(0xA2); 
				 break;
			case  0xA3:                     /*APP询问洗衣机状态*/
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
 * @brief  	判断串口数据的帧头和校验和是否正确
 *          串口接收完成标志
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
 * @brief  	判断串口是否接收到有效数据，若有效则拷贝到缓冲区中，并置位串
 *          口接收完成标志
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
 * @brief  	更改系统时钟和PLL时钟源为内部高速晶振，设置相关总线的频率
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
 * @brief  	计算二补数校验方式的校验和
 * @param  	uint8_t * data：指向待计算数据缓冲区的指针
 *          uint8_t len：待计算数据的长度
 * @retval  返回二补数校验和	
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
 * @brief  	判断二补数校验方式的校验和是否正确
 * @param  	uint8_t * data：指向待校验数据缓冲区的指针
 *          uint8_t len：待校验数据的长度
 * @retval 	校验成功返回0，失败返回2	
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
 * @brief  	中断优先级配置，设置串口1的中断优先级
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
 * @brief  	初始化串口1，设置串口接收、发送引脚，设置比特率等
 * @param  	USART_TypeDef* USARTx：串口号
 *          uint8_t *Data：指向发送缓冲区的指针
 *          uint16_t Len：发送的数据长度
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
* @brief  	初始化串口1，设置串口接收、发送引脚，设置比特率等
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void USART_INIT(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/**初始化串口1时钟、GPIOA时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	
	
	/*复位串口1*/
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
 * @brief  	获取CRC校验的一字节校验（公式为32位）
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
 * @brief  	获取洗衣机状态：获取洗衣机的运行/待机信号，许可/不许可信号
 * @param  	None
 * @retval 	None	
 * 
 ************************************************************************/
void get_washer_status(void)
{
	/*获取许可状态*/
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
	/*获取洗衣机状态*/
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
	/*实时状态判断*/
	if((my_washer.permission_status == NORMAL) && (my_washer.operation_status == NORMAL))
	{                                                          
			my_washer.now_washer_status = 0x00;            /*待机状态*/
	}
	else if((my_washer.permission_status == ABNORMAL) && (my_washer.operation_status == NORMAL) && (my_washer.think_washer_status != 0x01)) /*待机状态下的异常*/
	{
		my_washer.now_washer_status = 0x03;                            /*门被打开了*/
	}
	else if((my_washer.permission_status == ABNORMAL) && (my_washer.operation_status == ABNORMAL))
	{
		my_washer.now_washer_status = 0x01;                            /*处于运行中状态*/
	}
	
	if((my_washer.operation_status == NORMAL)&&(my_washer.think_washer_status == 0x01)&&(washer_work_flag == WASHWE_RUNING)) /*从洗衣状态回到待机状态*/
	{
				my_washer.think_washer_status = 0x02;                       /*洗衣完成*/
				washer_work_flag = WASHWE_END;
	}
}