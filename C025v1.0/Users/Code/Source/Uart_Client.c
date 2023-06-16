#include "main.h"

UINT16 g_u16UartClientCommuErrCnt = 0;         //SCI通信异常计数
UINT8  g_u8UartClientTxEnable = 0;
UINT8  g_u8UartClientTxFinishFlag = 0;

UINT8 g_u8UartClientTxBuff[USART_TX_BUF_LEN];

struct RS485MSG g_UartClientMsgPtr;


void UartClient_DataInit( struct RS485MSG *s ) {
	UINT16 i;

	s->ptr_no = 0;
	s->csr = RS485_STA_IDLE;
	s->enRs485CmdType = RS485_CMD_READ_REGS;
	for(i = 0; i < RS485_MAX_BUFFER_SIZE; i++) {
		s->u16Buffer[i] = 0;
	}
	for(i = 0; i < SCI_TX_BUF_LEN; i++) {
		g_u8UartClientTxBuff[i] = 0;
	}
}


void CRC8_Verify(struct RS485MSG *s) {
	UINT8 u8SciVerify;
	u8SciVerify = s->u16Buffer[s->ptr_no-1];
	if(u8SciVerify == CRC8((UINT8*)s->u16Buffer, s->ptr_no-1, CRC_KEY)) {		
		s ->AckType = RS485_ACK_POS;
	}
	else {
		s ->u16RdRegByteNum = 0;
		s ->AckType = RS485_ACK_NEG;
		s ->ErrorType = RS485_ERROR_CRC_ERROR;
	}
}


void Verify_Client(struct RS485MSG *s) {
	UINT8 u8SciVerify1,u8SciVerify2;
	u8SciVerify1 = s->u16Buffer[s->ptr_no-1];
	u8SciVerify2 = s->u16Buffer[2] + s->u16Buffer[3] + s->u16Buffer[9] + 1;
	if(u8SciVerify1 == u8SciVerify2) {		
		s ->AckType = RS485_ACK_POS;
	}
	else {
		s ->u16RdRegByteNum = 0;
		s ->AckType = RS485_ACK_NEG;
		s ->ErrorType = RS485_ERROR_CRC_ERROR;
	}
}


void UartClient_CRC_Verify(struct RS485MSG *s) {
	switch(s->enRs485CmdType) {
		case UART_CLIENT_CMD_0x01:
			CRC8_Verify(s);
			break;
		case UART_CLIENT_CMD_0x02:
			CRC8_Verify(s);			//如果有不同的方式，则分开，这里可以改
			break;
		default:
			s ->u16RdRegByteNum = 0;
			s ->AckType = RS485_ACK_NEG;
			s ->ErrorType = RS485_ERROR_NULL;
			break;
	}
}


//暂时不写
void UartClient_RxDataDeal_0x01(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	#if 0
	UINT8 test;
	test = s->u16Buffer[5];
	#endif
}


//暂时不写
void UartClient_RxDataDeal_0x02(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	#if 0
	UINT8 test;
	test = s->u16Buffer[5];
	#endif
}


void UartClient_AckDataDeal_0x01(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	UINT16	i;
	for(i = 0; i < 8; ++i) {
		t_u8BuffTemp[i] = s->u16Buffer[i+2];
	}
}

void UartClient_AckDataDeal_0x02(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	t_u8BuffTemp[0] = SeriesNum;
	//t_u8BuffTemp[1] = g_stCellInfoReport.u16Soc;
}


//不用改
void UartClient_DataDeal_0x01(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	UartClient_RxDataDeal_0x01(s, t_u8BuffTemp);
	UartClient_AckDataDeal_0x01(s, t_u8BuffTemp);
}


//不用改
void UartClient_DataDeal_0x02(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	UartClient_RxDataDeal_0x02(s, t_u8BuffTemp);
	UartClient_AckDataDeal_0x02(s, t_u8BuffTemp);
}


void UartClient_ACK_0x01(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	UINT8   i=0,j=0;	
	if(s->AckType == RS485_ACK_POS) {
		s->u16Buffer[i++] = UART_CLIENT_HEAD_ADDR;
		s->u16Buffer[i++] = s->enRs485CmdType + 0x20;		//收到A1,A2,返回C1,C2
		while(j < 8) {
			s->u16Buffer[i++] = t_u8BuffTemp[j++];
		}
		//s->u16Buffer[i++] = s->u16Buffer[2] + s->u16Buffer[3] + s->u16Buffer[9] + 1;
		s->u16Buffer[i++] = CRC8((UINT8*)s->u16Buffer, 10, CRC_KEY);
		s->u16Buffer[i++] = UART_CLIENT_END_VALUE;
	}
	else {
		s->u16Buffer[i++] = UART_CLIENT_HEAD_ADDR;
		s->u16Buffer[i++] = s->enRs485CmdType + 0x20;		//收到A1,A2,返回C1,C2
		//s->u16Buffer[i++] = 1;
		s->u16Buffer[i++] = CRC8((UINT8*)s->u16Buffer, 2, CRC_KEY);
		s->u16Buffer[i++] = UART_CLIENT_END_VALUE;
		++g_u16UartClientCommuErrCnt;						//不返回
	}
	s->AckLenth = i;
	s->ptr_no = 0;
	s->csr = RS485_STA_TX_COMPLETE; 
}


void UartClient_ACK_0x02(struct RS485MSG *s,UINT8 t_u8BuffTemp[]) {
	UINT8   i=0,j=0;	
	if(s->AckType == RS485_ACK_POS) {
		s->u16Buffer[i++] = UART_CLIENT_HEAD_ADDR;
		s->u16Buffer[i++] = s->enRs485CmdType + 0x20;		//收到A1,A2,返回C1,C2
		while(j < 8) {
			s->u16Buffer[i++] = t_u8BuffTemp[j++];
		}
		//s->u16Buffer[i++] = s->u16Buffer[2] + s->u16Buffer[3] + s->u16Buffer[9] + 1;
		s->u16Buffer[i++] = CRC8((UINT8*)s->u16Buffer, 10, CRC_KEY);
		s->u16Buffer[i++] = UART_CLIENT_END_VALUE;
	}
	else {
		s->u16Buffer[i++] = UART_CLIENT_HEAD_ADDR;
		s->u16Buffer[i++] = s->enRs485CmdType + 0x20;		//收到A1,A2,返回C1,C2
		//s->u16Buffer[i++] = 1;
		s->u16Buffer[i++] = CRC8((UINT8*)s->u16Buffer, 2, CRC_KEY);
		s->u16Buffer[i++] = UART_CLIENT_END_VALUE;
		++g_u16UartClientCommuErrCnt;						//不返回
	}
	s->AckLenth = i;
	s->ptr_no = 0;
	s->csr = RS485_STA_TX_COMPLETE; 
}


#if (defined _CLIENT_SCI1)

void InitUSART_UartClient(void) {
	CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE); //使能USART1时钟
	#if 0
	GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Fast);
	GPIO_Init(GPIOA, GPIO_Pin_3, GPIO_Mode_In_PU_No_IT);
	SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA,ENABLE);
	#endif
	GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Slow);	//TX
	GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_In_PU_No_IT);		//RX
	
	USART_Init(USART1,				  			//设置USART1
			  19200, 			  				//波特率设置
			  USART_WordLength_8b,	  			//数据长度设为8位
			  USART_StopBits_1, 	  			//1位停止位
			  USART_Parity_No,		  			//无校验
			  (USART_Mode_Rx | USART_Mode_Tx)); //设置为发送接收双模式

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1 , ENABLE);

    UartClient_DataInit(&g_UartClientMsgPtr);	
}


void UartClient_FaultChk(void) {
	UINT8 ClearRead;
	UINT8 FaultCnt = 0;
	
	if(USART1->SR&0x08) {		//接收溢出错误，RXNEIE或EIE使能产生中断，开
		ClearRead = USART1->SR;	//清除
		ClearRead = USART1->DR;
		FaultCnt++;
	}
	
	if(USART1->SR&0x04) {		//检测到噪声，默认开，不开的话CR3的ONEBIT置1，不开
								//USART_CR3的EIE使能中断
		ClearRead = USART1->SR; //清除
		ClearRead = USART1->DR;
		FaultCnt++;
	}

	if(USART1->SR&0x02) {		//帧错误，USART_CR3的EIE使能中断，开
		ClearRead = USART1->SR;	//清除
		ClearRead = USART1->DR;
		FaultCnt++;
	}

	if(USART1->SR&0x01) {		//校验错误标志 USART_CR2的PEIE使能该中断，不开
		ClearRead = USART1->SR;	//清除
		ClearRead = USART1->DR;
		FaultCnt++;
	}

	if(FaultCnt) {
		g_u16UartClientCommuErrCnt++;
	}	
}


void UartClient_Rx_Deal(struct RS485MSG *s) {
	USART1->CR2 &= ~(1<<5);						//禁止EUSART接收中断
	s->u16Buffer[s->ptr_no] = USART1->DR;
	//s->u16Buffer[s->ptr_no] = (UINT8)USART_ReceiveData9(USART1);	//带奇偶校验，舍弃奇偶奇偶校验位
																	//如果出现奇偶校验错误，会在中断函数处处理。
	if((s->ptr_no == 0) && (s->u16Buffer[0] != UART_CLIENT_HEAD_ADDR)) {
		s->ptr_no = 0;
		s->u16Buffer[0] = 0;
	}
	else {
		if(s->ptr_no == 1) {
			switch(s->u16Buffer[s->ptr_no]) {
				case UART_CLIENT_CMD_0x01:
					s->enRs485CmdType = UART_CLIENT_CMD_0x01;
					break;
				case UART_CLIENT_CMD_0x02:
					s->enRs485CmdType = UART_CLIENT_CMD_0x02;
					break;
				default:
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					s->u16Buffer[1] = 0;
					break;
			}
		}
		else if(s->ptr_no >= 2) {
			switch(s->enRs485CmdType) {
				case UART_CLIENT_CMD_0x01: 
					if (s->ptr_no == 11) {		//receive complete
						s->csr = RS485_STA_RX_COMPLETE;
						USART1->CR2 &= ~(1<<2);	//禁止接收
						USART1->CR2 &= ~(1<<5);	//禁止EUSART接收中断
					}
					break;
					
				case UART_CLIENT_CMD_0x02:
					if(s->ptr_no == 11) {		//兼容设计在这里改便可
						s->csr = RS485_STA_RX_COMPLETE;
						USART1->CR2 &= ~(1<<2);	//禁止接收
						USART1->CR2 &= ~(1<<5);	//禁止EUSART接收中断
					}
					break;
				default: {
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					break;
				}
			}
		}
		
		s->ptr_no++;
		if(s->ptr_no == RS485_MAX_BUFFER_SIZE) {
			s->ptr_no = 0;
			s->u16Buffer[0] = 0;
		}
	}
	
	USART1->CR2 |= (1<<5);	//使能EUSART接收中断
}

void UartClient_Tx_Deal(struct RS485MSG *s ) {
	if(0 == g_u8UartClientTxEnable) {
		return;
	}

	if(g_u16UartClientCommuErrCnt) {	//出现奇偶校验错误也得把数据全部接收完，然后不回复
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8UartClientTxFinishFlag = 1;
		g_u8UartClientTxEnable = 0;
		g_u16UartClientCommuErrCnt = 0;
		return;
	}
	
	while(!((USART1->SR)&(1<<7)));				// 1<<6 也可以
	if( s->ptr_no < s -> AckLenth) {
        USART1->DR = s->u16Buffer[s->ptr_no];	//load data
        //USART_SendData9(USART1, Usart_9bitOddEvenData_Frame(s->u16Buffer[s->ptr_no], ODD));	//不知道是否真的需要自己计算，先试试
        
		s->ptr_no++;
	}
	else {
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8UartClientTxFinishFlag = 1;
		g_u8UartClientTxEnable = 0;
	}
}


void App_UartClient_Updata(struct RS485MSG *s) {
	switch(s->csr) {
		//IDLE-空闲态，保持50ms后使能接收（物理层）receive set
		case RS485_STA_IDLE: {
			break;
		}
		//receive complete, to deal the receive data
		case RS485_STA_RX_COMPLETE: {
			USART1->CR2 &= ~(1<<5);		//禁止产生中断
			UartClient_CRC_Verify(s);
			if(s ->AckType == RS485_ACK_POS) {
				switch(s->enRs485CmdType) {
					case UART_CLIENT_CMD_0x01:
						UartClient_DataDeal_0x01(s, g_u8UartClientTxBuff);
						break;
					case UART_CLIENT_CMD_0x02:
						UartClient_DataDeal_0x02(s, g_u8UartClientTxBuff);
						break;
					default:
						s ->u16RdRegByteNum = 0;
						s ->AckType = RS485_ACK_NEG;
						s ->ErrorType = RS485_ERROR_NULL;
						break;
				}
			}
			s->csr = RS485_STA_RX_OK;	//receive the correct data, switch to transmit wait 50ms
			break;              		//下一轮再来
		}
		//receive ok, to transmit wait 50ms
		case RS485_STA_RX_OK: {
			switch(s->enRs485CmdType) {
				case UART_CLIENT_CMD_0x01:
					UartClient_ACK_0x01(s, g_u8UartClientTxBuff);
					break;
				case UART_CLIENT_CMD_0x02:
					UartClient_ACK_0x02(s, g_u8UartClientTxBuff);
					break;
				default:				//这个defualt不用加错误操作
					break;
			}
			USART1->CR2 |= (1<<3); 		//使能发送
			g_u8UartClientTxEnable = 1;
		}
		//transmit complete, to switch receive wait 20ms
		case RS485_STA_TX_COMPLETE: {
			if(g_u8UartClientTxFinishFlag) {
				s->csr = RS485_STA_IDLE;
				s->u16Buffer[0] = 0;
				s->u16Buffer[1] = 0;
				s->u16Buffer[2] = 0;
				s->u16Buffer[3] = 0;
				g_u8UartClientTxFinishFlag = 0;
				s->ptr_no = 0;
				USART1->CR2 |= (1<<2);	//使能接收
				USART1->CR2 |= (1<<5);	//使能EUSART接收中断
				//TXSTA1bits.TXEN = 0;    //禁止发送，不能禁止，否则通信很容易断
				g_u8UartClientTxEnable = 0;
			}
			break;
		}
		
		default: {
			s->csr = RS485_STA_IDLE;
			break;
		}
	}

	UartClient_Tx_Deal(s);
	//UartClient_FaultChk();		//STM系列的写在这里毫无用处
}


#elif (defined _CLIENT_SCI2)

void InitUSART_UartClient(void) {
	CLK_PeripheralClockConfig(CLK_Peripheral_USART2, ENABLE); 	//使能USART1时钟

	GPIO_Init(GPIOE, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Slow);	//TX
	GPIO_Init(GPIOE, GPIO_Pin_3, GPIO_Mode_In_PU_No_IT);		//RX

	USART_Init(USART2,				  			//设置USART1
			  19200, 			  				//波特率设置
			  USART_WordLength_8b,	  			//数据长度设为9位，包含校验位
			  USART_StopBits_1, 	  			//1位停止位
			  USART_Parity_No,		  			//无校验
			  (USART_Mode_Rx | USART_Mode_Tx)); //设置为发送接收双模式
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2 , ENABLE);
    
    UartClient_DataInit(&g_UartClientMsgPtr);	
}


void UartClient_FaultChk(void) {
	UINT8 ClearRead;
	UINT8 FaultCnt = 0;
	
	if(USART2->SR&0x08) {		//接收溢出错误，RXNEIE或EIE使能产生中断，开
		ClearRead = USART2->SR;	//清除
		ClearRead = USART2->DR;
		FaultCnt++;
	}
	
	if(USART2->SR&0x04) {		//检测到噪声，默认开，不开的话CR3的ONEBIT置1，不开
								//USART_CR3的EIE使能中断
		ClearRead = USART2->SR; //清除
		ClearRead = USART2->DR;
		FaultCnt++;
	}

	if(USART2->SR&0x02) {		//帧错误，USART_CR3的EIE使能中断，开
		ClearRead = USART2->SR;	//清除
		ClearRead = USART2->DR;
		FaultCnt++;
	}

	if(USART2->SR&0x01) {		//校验错误标志 USART_CR2的PEIE使能该中断，不开
		ClearRead = USART2->SR;	//清除
		ClearRead = USART2->DR;
		FaultCnt++;
	}
	
	if(FaultCnt) {
		g_u16UartClientCommuErrCnt++;
	}

}


void UartClient_Rx_Deal(struct RS485MSG *s) {
	USART2->CR2 &= ~(1<<5);						//禁止EUSART接收中断
	s->u16Buffer[s->ptr_no] = USART2->DR;
	//s->u16Buffer[s->ptr_no] = (UINT8)USART_ReceiveData9(USART2);	//带奇偶校验，舍弃奇偶奇偶校验位
																	//如果出现奇偶校验错误，会在中断函数处处理。
	if((s->ptr_no == 0) && (s->u16Buffer[0] != UART_CLIENT_HEAD_ADDR)) {
		s->ptr_no = 0;
		s->u16Buffer[0] = 0;
	}
	else {
		if(s->ptr_no == 1) {
			switch(s->u16Buffer[s->ptr_no]) {
				case UART_CLIENT_CMD_0x01:
					s->enRs485CmdType = UART_CLIENT_CMD_0x01;
					break;
				case UART_CLIENT_CMD_0x02:
					s->enRs485CmdType = UART_CLIENT_CMD_0x02;
					break;
				default:
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					s->u16Buffer[1] = 0;
					break;
			}
		}
		else if(s->ptr_no >= 2) {
			switch(s->enRs485CmdType) {
				case UART_CLIENT_CMD_0x01: 
					if (s->ptr_no == 11) {		//receive complete
						s->csr = RS485_STA_RX_COMPLETE;
						USART2->CR2 &= ~(1<<2);	//禁止接收
						USART2->CR2 &= ~(1<<5);	//禁止EUSART接收中断
					}
					break;
					
				case UART_CLIENT_CMD_0x02:
					if(s->ptr_no == 11) {		//兼容设计在这里改便可
						s->csr = RS485_STA_RX_COMPLETE;
						USART2->CR2 &= ~(1<<2);	//禁止接收
						USART2->CR2 &= ~(1<<5);	//禁止EUSART接收中断
					}
					break;
				default: {
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					break;
				}
			}
		}
		
		s->ptr_no++;
		if(s->ptr_no == RS485_MAX_BUFFER_SIZE) {
			s->ptr_no = 0;
			s->u16Buffer[0] = 0;
		}
	}
	
	USART2->CR2 |= (1<<5);	//使能EUSART接收中断
}


void UartClient_Tx_Deal(struct RS485MSG *s ) {
	if(0 == g_u8UartClientTxEnable) {
		return;
	}

	if(g_u16UartClientCommuErrCnt) {	//出现奇偶校验错误也得把数据全部接收完，然后不回复
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8UartClientTxFinishFlag = 1;
		g_u8UartClientTxEnable = 0;
		g_u16UartClientCommuErrCnt = 0;
		return;
	}
	
	while(!((USART2->SR)&(1<<7)));				// 1<<6 也可以
	if( s->ptr_no < s -> AckLenth) {
        USART2->DR = s->u16Buffer[s->ptr_no];	//load data
        //USART_SendData9(USART2, Usart_9bitOddEvenData_Frame(s->u16Buffer[s->ptr_no], ODD));	//不知道是否真的需要自己计算，先试试
        
		s->ptr_no++;
	}
	else {
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8UartClientTxFinishFlag = 1;
		g_u8UartClientTxEnable = 0;
	}
}


void App_UartClient_Updata(struct RS485MSG *s) {
	switch(s->csr) {
		//IDLE-空闲态，保持50ms后使能接收（物理层）receive set
		case RS485_STA_IDLE: {
			break;
		}
		//receive complete, to deal the receive data
		case RS485_STA_RX_COMPLETE: {
			USART2->CR2 &= ~(1<<5);		//禁止产生中断
			UartClient_CRC_Verify(s);
			if(s ->AckType == RS485_ACK_POS) {
				switch(s->enRs485CmdType) {
					case UART_CLIENT_CMD_0x01:
						UartClient_DataDeal_0x01(s, g_u8UartClientTxBuff);
						break;
					case UART_CLIENT_CMD_0x02:
						UartClient_DataDeal_0x02(s, g_u8UartClientTxBuff);
						break;
					default:
						s ->u16RdRegByteNum = 0;
						s ->AckType = RS485_ACK_NEG;
						s ->ErrorType = RS485_ERROR_NULL;
						break;
				}
			}
			s->csr = RS485_STA_RX_OK;	//receive the correct data, switch to transmit wait 50ms
			break;              		//下一轮再来
		}
		//receive ok, to transmit wait 50ms
		case RS485_STA_RX_OK: {
			switch(s->enRs485CmdType) {
				case UART_CLIENT_CMD_0x01:
					UartClient_ACK_0x01(s, g_u8UartClientTxBuff);
					break;
				case UART_CLIENT_CMD_0x02:
					UartClient_ACK_0x02(s, g_u8UartClientTxBuff);
					break;
				default:				//这个defualt不用加错误操作
					break;
			}
			USART2->CR2 |= (1<<3); 		//使能发送
			g_u8UartClientTxEnable = 1;
		}
		//transmit complete, to switch receive wait 20ms
		case RS485_STA_TX_COMPLETE: {
			if(g_u8UartClientTxFinishFlag) {
				s->csr = RS485_STA_IDLE;
				s->u16Buffer[0] = 0;
				s->u16Buffer[1] = 0;
				s->u16Buffer[2] = 0;
				s->u16Buffer[3] = 0;
				g_u8UartClientTxFinishFlag = 0;
				s->ptr_no = 0;
				USART2->CR2 |= (1<<2);	//使能接收
				USART2->CR2 |= (1<<5);	//使能EUSART接收中断
				//TXSTA1bits.TXEN = 0;    //禁止发送，不能禁止，否则通信很容易断
				g_u8UartClientTxEnable = 0;
			}
			break;
		}
		
		default: {
			s->csr = RS485_STA_IDLE;
			break;
		}
	}

	UartClient_Tx_Deal(s);
	//UartClient_FaultChk();		//STM系列的写在这里毫无用处
}

#elif (defined _CLIENT_SCI3)

void InitUSART_UartClient(void) {
	CLK_PeripheralClockConfig(CLK_Peripheral_USART3, ENABLE); 	//使能USART1时钟

	GPIO_Init(GPIOE, GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);	//TX
	GPIO_Init(GPIOE, GPIO_Pin_6, GPIO_Mode_In_PU_No_IT);		//RX

	USART_Init(USART3,				  			//设置USART1
			  19200, 			  				//波特率设置
			  USART_WordLength_8b,	  			//数据长度设为9位，包含校验位
			  USART_StopBits_1, 	  			//1位停止位
			  USART_Parity_No,		  			//无校验
			  (USART_Mode_Rx | USART_Mode_Tx)); //设置为发送接收双模式
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3 , ENABLE);	
}


void UartClient_FaultChk(void) {
	UINT8 ClearRead;
	UINT8 FaultCnt = 0;
	
	if(USART3->SR&0x08) {		//接收溢出错误，RXNEIE或EIE使能产生中断，开
		ClearRead = USART3->SR;	//清除
		ClearRead = USART3->DR;
		FaultCnt++;
	}
	
	if(USART3->SR&0x04) {		//检测到噪声，默认开，不开的话CR3的ONEBIT置1，不开
								//USART_CR3的EIE使能中断
		ClearRead = USART3->SR; //清除
		ClearRead = USART3->DR;
		FaultCnt++;
	}

	if(USART3->SR&0x02) {		//帧错误，USART_CR3的EIE使能中断，开
		ClearRead = USART3->SR;	//清除
		ClearRead = USART3->DR;
		FaultCnt++;
	}

	if(USART3->SR&0x01) {		//校验错误标志 USART_CR2的PEIE使能该中断，不开
		ClearRead = USART3->SR;	//清除
		ClearRead = USART3->DR;
		FaultCnt++;
	}
	
	if(FaultCnt) {
		g_u16UartClientCommuErrCnt++;
	}
}


void UartClient_Rx_Deal(struct RS485MSG *s) {
	USART3->CR2 &= ~(1<<5);						//禁止EUSART接收中断
	s->u16Buffer[s->ptr_no] = USART3->DR;
	//s->u16Buffer[s->ptr_no] = (UINT8)USART_ReceiveData9(USART3);	//带奇偶校验，舍弃奇偶奇偶校验位
																	//如果出现奇偶校验错误，会在中断函数处处理。
	if((s->ptr_no == 0) && (s->u16Buffer[0] != UART_CLIENT_HEAD_ADDR)) {
		s->ptr_no = 0;
		s->u16Buffer[0] = 0;
	}
	else {
		if(s->ptr_no == 1) {
			switch(s->u16Buffer[s->ptr_no]) {
				case UART_CLIENT_CMD_0x01:
					s->enRs485CmdType = UART_CLIENT_CMD_0x01;
					break;
				case UART_CLIENT_CMD_0x02:
					s->enRs485CmdType = UART_CLIENT_CMD_0x02;
					break;
				default:
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					s->u16Buffer[1] = 0;
					break;
			}
		}
		else if(s->ptr_no >= 2) {
			switch(s->enRs485CmdType) {
				case UART_CLIENT_CMD_0x01: 
					if (s->ptr_no == 11) {		//receive complete
						s->csr = RS485_STA_RX_COMPLETE;
						USART3->CR2 &= ~(1<<2);	//禁止接收
						USART3->CR2 &= ~(1<<5);	//禁止EUSART接收中断
					}
					break;
					
				case UART_CLIENT_CMD_0x02:
					if(s->ptr_no == 11) {		//兼容设计在这里改便可
						s->csr = RS485_STA_RX_COMPLETE;
						USART3->CR2 &= ~(1<<2);	//禁止接收
						USART3->CR2 &= ~(1<<5);	//禁止EUSART接收中断
					}
					break;
				default: {
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					break;
				}
			}
		}
		
		s->ptr_no++;
		if(s->ptr_no == RS485_MAX_BUFFER_SIZE) {
			s->ptr_no = 0;
			s->u16Buffer[0] = 0;
		}
	}
	
	USART3->CR2 |= (1<<5);	//使能EUSART接收中断
}


void UartClient_Tx_Deal(struct RS485MSG *s ) {
	if(0 == g_u8UartClientTxEnable) {
		return;
	}

	if(g_u16UartClientCommuErrCnt) {	//出现奇偶校验错误也得把数据全部接收完，然后不回复
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8UartClientTxFinishFlag = 1;
		g_u8UartClientTxEnable = 0;
		g_u16UartClientCommuErrCnt = 0;
		return;
	}
	
	while(!((USART3->SR)&(1<<7)));				// 1<<6 也可以
	if( s->ptr_no < s -> AckLenth) {
        USART3->DR = s->u16Buffer[s->ptr_no];	//load data
        //USART_SendData9(USART3, Usart_9bitOddEvenData_Frame(s->u16Buffer[s->ptr_no], ODD));	//不知道是否真的需要自己计算，先试试
        
		s->ptr_no++;
	}
	else {
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8UartClientTxFinishFlag = 1;
		g_u8UartClientTxEnable = 0;
	}
}


void App_UartClient_Updata(struct RS485MSG *s) {
	switch(s->csr) {
		//IDLE-空闲态，保持50ms后使能接收（物理层）receive set
		case RS485_STA_IDLE: {
			break;
		}
		//receive complete, to deal the receive data
		case RS485_STA_RX_COMPLETE: {
			USART3->CR2 &= ~(1<<5);		//禁止产生中断
			UartClient_CRC_Verify(s);
			if(s ->AckType == RS485_ACK_POS) {
				switch(s->enRs485CmdType) {
					case UART_CLIENT_CMD_0x01:
						UartClient_DataDeal_0x01(s, g_u8UartClientTxBuff);
						break;
					case UART_CLIENT_CMD_0x02:
						UartClient_DataDeal_0x02(s, g_u8UartClientTxBuff);
						break;
					default:
						s ->u16RdRegByteNum = 0;
						s ->AckType = RS485_ACK_NEG;
						s ->ErrorType = RS485_ERROR_NULL;
						break;
				}
			}
			s->csr = RS485_STA_RX_OK;	//receive the correct data, switch to transmit wait 50ms
			break;              		//下一轮再来
		}
		//receive ok, to transmit wait 50ms
		case RS485_STA_RX_OK: {
			switch(s->enRs485CmdType) {
				case UART_CLIENT_CMD_0x01:
					UartClient_ACK_0x01(s, g_u8UartClientTxBuff);
					break;
				case UART_CLIENT_CMD_0x02:
					UartClient_ACK_0x02(s, g_u8UartClientTxBuff);
					break;
				default:				//这个defualt不用加错误操作
					break;
			}
			USART3->CR2 |= (1<<3); 		//使能发送
			g_u8UartClientTxEnable = 1;
		}
		//transmit complete, to switch receive wait 20ms
		case RS485_STA_TX_COMPLETE: {
			if(g_u8UartClientTxFinishFlag) {
				s->csr = RS485_STA_IDLE;
				s->u16Buffer[0] = 0;
				s->u16Buffer[1] = 0;
				s->u16Buffer[2] = 0;
				s->u16Buffer[3] = 0;
				g_u8UartClientTxFinishFlag = 0;
				s->ptr_no = 0;
				USART3->CR2 |= (1<<2);	//使能接收
				USART3->CR2 |= (1<<5);	//使能EUSART接收中断
				//TXSTA1bits.TXEN = 0;    //禁止发送，不能禁止，否则通信很容易断
				g_u8UartClientTxEnable = 0;
			}
			break;
		}
		
		default: {
			s->csr = RS485_STA_IDLE;
			break;
		}
	}

	UartClient_Tx_Deal(s);
	//UartClient_FaultChk();		//STM系列的写在这里毫无用处
}

#endif

