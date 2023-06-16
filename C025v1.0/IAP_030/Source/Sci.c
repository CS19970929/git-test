#include "main.h"

//���ȵ����⣬��ô���APP��UPPER���ȵ����⣬����REG����������
// SCI_485 Message Structure
// SCI_485 Message Structure
struct RS485MSG {
	UINT16	ptr_no;          	// Word stating what state msg is in
	UINT8	csr;          		// I2C address of slave msg is intended for
	UINT16	u16RdRegStartAddr;	// read reg start addr
	UINT16	u16RdRegStartAddrActure;
	UINT8	u16RdRegByteNum;    // read byte lenth
	UINT8	AckLenth;			// ack byte lenth
	UINT8	AckType;			// ack type
	UINT8	ErrorType;			// error type
	UINT8 	u16Buffer[RS485_MAX_BUFFER_SIZE];    // Array holding msg data - max that
	enum RS485_CMD_E enRs485CmdType;
};

/*************************SCI1 Global Variables****************************/
uint16_t g_u16SCI1CommuErrCnt = 0;         //SCIͨ���쳣����
uint8_t  g_u8SCI1TxEnable = 0;
uint8_t	g_u8SCI1TxFinishFlag = 0;
struct RS485MSG g_stSCI1CurrentMsgPtr;


uint16_t g_u16SCI2CommuErrCnt = 0;         //SCIͨ���쳣����
uint8_t  g_u8SCI2TxEnable = 0;
uint8_t	g_u8SCI2TxFinishFlag = 0;
struct RS485MSG g_stSCI2CurrentMsgPtr;


uint8_t	g_u8SCITxBuff[SCI_TX_BUF_LEN];
uint8_t u8FlashReceiveCnt = 0;

UINT16 FlashReadOneHalfWord(UINT32 faddr) {
	return *(vu16*)faddr; 
}

FLASH_Status FlashWriteOneHalfWord(uint32_t StartAddr,uint16_t Buffer) {
	FLASH_Status result;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	while(FLASH_ErasePage(StartAddr) != FLASH_COMPLETE);
	result = FLASH_ProgramHalfWord(StartAddr,Buffer);
	FLASH_Lock();
	return result;
}

void APPFlashErase(uint16_t Sector)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	while(FLASH_ErasePage(FLASH_ADDR_APP_START + Sector * 1024) != FLASH_COMPLETE);
	FLASH_Lock();
}

void FlashWrite(uint32_t StartAddr,uint8_t* Buffer,uint16_t Length)
{
	uint16_t TempBuffer = 0;
	uint16_t i;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	for(i = 0; i < Length; i += 2)
	{
		TempBuffer = Buffer[i] | 0xFF00;
		if( i + 1 < Length)
		{
			TempBuffer &= ((Buffer[i + 1] << 8) | 0x00FF);
		}
		FLASH_ProgramHalfWord(StartAddr + i,TempBuffer);
	}
	FLASH_Lock();
}

uint16_t Sci_CRC16RTU( uint8_t * pszBuf, uint16_t unLength)
{
	uint16_t CRCC=0XFFFF;
	uint32_t CRC_count;

	for(CRC_count=0;CRC_count<unLength;CRC_count++)
	{
		int i;

		CRCC=CRCC^*(pszBuf+CRC_count);

		for(i=0;i<8;i++)
		{
			if(CRCC&1)
			{
				CRCC>>=1;
				CRCC^=0xA001;
			}
			else
			{ 
                CRCC>>=1;
			}
				
		}
	}

	return CRCC;
}

void CRC_verify( struct RS485MSG *s) {
	uint16_t u16SciVerify;
	uint16_t t_u16FrameLenth;
	
	t_u16FrameLenth = s->ptr_no - 2;
	u16SciVerify = s->u16Buffer[t_u16FrameLenth] + (s->u16Buffer[t_u16FrameLenth+1] << 8);
	if(u16SciVerify == Sci_CRC16RTU(( uint8_t * )s->u16Buffer, t_u16FrameLenth)) {		
		s ->AckType = RS485_ACK_POS;
	}
	else {
		s ->u16RdRegByteNum = 0;
		s ->AckType = RS485_ACK_NEG;
		s ->ErrorType = RS485_ERROR_CRC_ERROR;
	}
}

void Sci_DataInit( struct RS485MSG *s) {
	s->csr = RS485_STA_IDLE;
	s->enRs485CmdType = RS485_CMD_READ_REGS;
}


void Sci_ReadRegs_Data( struct RS485MSG *s,UINT8 t_u8BuffTemp[])
{
	UINT16	u16SciTemp;
	UINT16	i;

    i = 0;
	/*
	for(j = 0;j < 34;j++)
	{
		u16SciTemp = 0;
		t_u8BuffTemp[i++] = (u16SciTemp>>8)& 0x00FF;
		t_u8BuffTemp[i++] = u16SciTemp & 0x00FF;
	}
	
	u16SciTemp = 0;
	t_u8BuffTemp[i++] = (u16SciTemp>>8)& 0x00FF;
	t_u8BuffTemp[i++] = u16SciTemp & 0x00FF;

	u16SciTemp = 0;
	t_u8BuffTemp[i++] = (u16SciTemp>>8)& 0x00FF;
	t_u8BuffTemp[i++] = u16SciTemp & 0x00FF;
	*/

	u16SciTemp = 0;
	t_u8BuffTemp[i++] = (u16SciTemp>>8)& 0x00FF;
	t_u8BuffTemp[i++] = u16SciTemp & 0x00FF;
}


void Sci_ReadRegs_ACK( struct RS485MSG *s ) {
	UINT8 i;
	UINT16	u16SciTemp;	
	if(s->AckType == RS485_ACK_POS) {
		if(s->u16RdRegStartAddrActure >= RS485_ADDR_RO_START) {
			//Sci_ReadRegs_Data(s,g_u8SCITxBuff);
		}
		//ͷ�룬ǰ�����ֽڱ��ֲ���
		s->u16Buffer[0] = (s->u16Buffer[0] != 0)?RS485_SLAVE_ADDR:s->u16Buffer[0];
		s->u16Buffer[1] = s->enRs485CmdType;
		s->u16Buffer[2] = s->u16RdRegByteNum;
		//����
		/*
		for(i = 0; i < (s->u16RdRegByteNum); i++ ) {
			s->u16Buffer[i+3] = g_u8SCITxBuff[i + ((s->u16RdRegStartAddr)<<1)];
		}
		*/
		i =  s->u16RdRegByteNum + 3;
	}
	else {
		i = 1;
		s->u16Buffer[i++] = s->enRs485CmdType | 0x80;
		s->u16Buffer[i++] = s->ErrorType;
	}
	u16SciTemp = Sci_CRC16RTU(( UINT8 * )s->u16Buffer, i);
	s->u16Buffer[i++] = u16SciTemp & 0x00FF;
	s->u16Buffer[i++] = u16SciTemp >> 8;
	s->AckLenth = i;
	
	s->ptr_no = 0;
	s->csr = RS485_STA_TX_COMPLETE; 
}


void Sci_ReadRegsDecode( struct RS485MSG *s ) {

	UINT16 t_u16Temp;

	t_u16Temp = s->u16Buffer[3] + (s->u16Buffer[2] << 8);
	s->u16RdRegStartAddrActure = t_u16Temp;

	if(t_u16Temp == (RS485_ADDR_RO_START + 0x0050)) {//13��
		t_u16Temp -= (RS485_ADDR_RO_START + 80-22-10-2);
	}
	else {
		t_u16Temp = 0;
	}
	s ->u16RdRegStartAddr = t_u16Temp;
	s ->u16RdRegByteNum = (s->u16Buffer[5] + (s->u16Buffer[4] << 8))<<1;
}

void Sci_WrRegsDecode(struct RS485MSG *s) {
	uint16_t	u16WrRegNum;
	uint16_t	u16SciRegStartAddr;

	u16SciRegStartAddr = s->u16Buffer[3] + (s->u16Buffer[2] << 8);
	u16WrRegNum = s->u16Buffer[5] + (s->u16Buffer[4] << 8);
	switch(u16SciRegStartAddr) {
		case RS485_CMD_ADDR_FLASH_CONNECT:
			if(u16WrRegNum == 1) {
				
			}
			else {
				s ->AckType = RS485_ACK_NEG;
				s ->ErrorType = RS485_ERROR_CMD_INVALID;
			}
			break;
			
		case RS485_CMD_ADDR_FLASH_UPGRATE:
			if(u16WrRegNum <= 1033) {
				APPFlashErase(u8FlashReceiveCnt);
				//���������ֵ�᲻�������
				FlashWrite(FLASH_ADDR_APP_START + 1024*u8FlashReceiveCnt, &s->u16Buffer[7], 1024);
				++u8FlashReceiveCnt;		//���˼ӣ������������ɣ�
			}
			else {
				s ->AckType = RS485_ACK_NEG;
				s ->ErrorType = RS485_ERROR_CMD_INVALID;
			}
			break;
			
		case RS485_CMD_ADDR_FLASH_COMPLETE:
			if(u16WrRegNum == 1) {
				FlashWriteOneHalfWord(FLASH_ADDR_UPDATE_FLAG, FLASH_TO_APP_VALUE);
				u8FlagUdFinishE2PROM = 1;
			}
			else {
				s ->AckType = RS485_ACK_NEG;
				s ->ErrorType = RS485_ERROR_CMD_INVALID;
			}
			break;
		

		default:
			s ->AckType = RS485_ACK_NEG;
			s ->ErrorType = RS485_ERROR_CMD_INVALID;
			break;
	}
}



void Sci_WrReg_s_Decode_ACK(struct RS485MSG *s) {
    uint8_t i;
	uint16_t	u16SciTemp;

	if( s->AckType == RS485_ACK_POS) {
		i = 6;
	}
	else {
		i = 1;
		s->u16Buffer[i++] = s->enRs485CmdType | 0x80;
		s->u16Buffer[i++] = s->ErrorType;
	}
	
	u16SciTemp = Sci_CRC16RTU(( uint8_t * )s->u16Buffer, (uint16_t)i);
	s->u16Buffer[i++] = u16SciTemp & 0x00FF;
	s->u16Buffer[i++] = u16SciTemp >> 8;
	s->AckLenth = i;

	s->ptr_no = 0;
	s->csr = RS485_STA_TX_COMPLETE;
}


#if (defined _COMMOM_UPPER_SCI1)
//���ڳ�ʼ������
void InitUSART1_CommonUpper(void) {
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);		//����USART1����ʱ��
	RCC->AHBENR |= 1<<17;										//����GPIOA������ʱ��
	//Enable the USART1 Interrupt(ʹ��USART1�ж�)
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPriority =0;   	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 


    //USART1_TX -> PA9 , USART1_RX -> PA10
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);		//030��AF����ڷ�reg��datasheet��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);                                     
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);        

	//���ڳ�ʼ��
    USART_InitStructure.USART_BaudRate = 19200;//���ô��ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��������λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//����ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����Ч��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//���ù���ģʽ
    USART_Init(USART1, &USART_InitStructure); //������ṹ��

	USART1->CR3 |= 1<<0;	//EIE����֡�����жϣ�ͬʱ���������ж�
	USART1->CR3 |= 1<<11;	//δ��ʹ��ǰ��д����ֹ�����ж�	

    USART_Cmd(USART1, ENABLE);//ʹ�ܴ���1
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);		   //ʹ�ܽ����ж�

	Sci_DataInit(&g_stSCI1CurrentMsgPtr);
}

void Sci1_CommonUpper_FaultChk(void) {
	UINT8 FaultCnt = 0;

	if(USART1->ISR&0x08) {		//�����������RXNEIE��EIEʹ�ܲ����жϣ���
		USART1->ICR |= 1<<3;	//���
		FaultCnt++;
	}

	if(USART1->ISR&0x04) {		//��⵽������Ĭ�Ͽ��������Ļ�CR3��ONEBIT��1������
								//USART_CR3��EIEʹ���ж�
		USART1->ICR |= 1<<2;	//���
		FaultCnt++;
	}

	if(USART1->ISR&0x02) {		//֡����USART_CR3��EIEʹ���жϣ���
		USART1->ICR |= 1<<1;	//���
		FaultCnt++;
	}

	if(USART1->ISR&0x01) {		//У������־ USART_CR1��PEIEʹ�ܸ��жϣ�����
		USART1->ICR |= 1<<0;	//���
		FaultCnt++;
	}
	if(FaultCnt) {
		g_u16SCI1CommuErrCnt++;
	}
}


void Sci1_CommonUpper_Rx_Deal(struct RS485MSG *s ) {
	USART1->CR1 &= ~(1<<5);		//�������Ǿ仰��ѡһ
	s->u16Buffer[s->ptr_no] = USART1->RDR; 				// ��RXFIFO �ж�ȡ���յ�������
	if((s->ptr_no == 0) && (s->u16Buffer[0] != RS485_SLAVE_ADDR )&& ( s->u16Buffer[0] != RS485_BROADCAST_ADDR )) {
		s->ptr_no = 0;
		s->u16Buffer[0] = 0;
	}
	else {
		if(s->ptr_no == 1) {
			switch(s->u16Buffer[s->ptr_no]) {
				case RS485_CMD_READ_REGS:
					s->enRs485CmdType = RS485_CMD_READ_REGS;
					s->ptr_no++;
					break;
					/*
				case RS485_CMD_WRITE_REG:
					s->enRs485CmdType = RS485_CMD_WRITE_REG;
					s->ptr_no++;
					break;
					*/
				case RS485_CMD_WRITE_REGS:
					s->enRs485CmdType = RS485_CMD_WRITE_REGS;
					s->ptr_no++;
					break;
				default:
					s->ptr_no = 0;
					s->u16Buffer[0] = 0;
					s->u16Buffer[1] = 0;
					break;
			}
		}
		else if(s->ptr_no > 2) {
			switch(s->enRs485CmdType) {
				case RS485_CMD_READ_REGS:
				//case RS485_CMD_WRITE_REG: 
				{
					if (s->ptr_no == 7)	{//	receive complete
						s->csr = RS485_STA_RX_COMPLETE;
						//RCSTA1bits.CREN = 0;  //��ֹ����
						//RC1IE = 0;			// ��ֹEUSART2 �����ж�
						USART1->CR1 &= ~(1<<2);
						USART1->CR1 &= ~(1<<5);	
					}
					break;
				}

				case RS485_CMD_WRITE_REGS:
				{	
					if(s->ptr_no >= 7) {
						if((s->u16Buffer[6] != 0 && s->ptr_no == (s->u16Buffer[6] + 8))
							||(s->u16Buffer[6] == 0 && s->ptr_no == s->u16Buffer[5]+(s->u16Buffer[4]<<8)+8)) {
							s->csr = RS485_STA_RX_COMPLETE;
							USART1->CR1 &= ~(1<<2);
							USART1->CR1 &= ~(1<<5);
						}
					}
					break;
				}
				default:
					//s->ptr_no = 0;
					//s->u16Buffer[0] = 0;
					break;
			}
			
			s->ptr_no++;

			if(s->ptr_no >= RS485_MAX_BUFFER_SIZE) {
				s->ptr_no = 0;
				s->u16Buffer[0] = 0;
			}
		}
		else {
			s->ptr_no++;
		}		
	}

	USART1->CR1 |= (1<<5);
}


/*=================================================================
* FUNCTION: Sci2_Tx_Deal
* PURPOSE : �������ݷ���
* INPUT:    void
*
* RETURN:   void
*
* CALLS:    void
*
* CALLED BY:SCI_TX_isr()
*
*=================================================================*/
void Sci1_CommonUpper_Tx_Deal(struct RS485MSG *s ) {
	if(0 == g_u8SCI1TxEnable) {
		return;
	}
	if(g_u16SCI1CommuErrCnt) {	//���ִ���Ҳ�ð�����ȫ�������꣬Ȼ�󲻻ظ�
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8SCI1TxFinishFlag = 1;
		g_u8SCI1TxEnable = 0;
		g_u16SCI1CommuErrCnt = 0;
		return;
	}
	while(!((USART1->ISR)&(1<<7)));				//Waitting Transmit data register empty
	if( s->ptr_no < s -> AckLenth) {
        USART1->TDR = s->u16Buffer[s->ptr_no];	//load data
		s->ptr_no++;
	}
	else {
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8SCI1TxFinishFlag = 1;
		g_u8SCI1TxEnable = 0;
		if(u8FlagUdFinishE2PROM) {
			u8FlagUdFinishE2PROM = 0;
			u8FlagUdFinish = 1;
		}
	}
}


void App_CommonUpper_Sci1(struct RS485MSG *s ) {
	switch(s->csr) {
		//IDLE-����̬������50ms��ʹ�ܽ��գ�����㣩receive set
		case RS485_STA_IDLE: {
			break;
		}
		//receive complete, to deal the receive data
		case RS485_STA_RX_COMPLETE: {
			//RC1IE = 0;// ��ֹEUSART1 �����ж�
			USART1->CR1 &= ~(1<<5);
			CRC_verify(s);
			if(s ->AckType == RS485_ACK_POS) {
				switch(s->enRs485CmdType) {
					case RS485_CMD_READ_REGS:
						Sci_ReadRegsDecode(s);
						break;;
					case RS485_CMD_WRITE_REGS:
						Sci_WrRegsDecode(s);
						break;
					default:
						s ->u16RdRegByteNum = 0;
						s ->AckType = RS485_ACK_NEG;
						s ->ErrorType = RS485_ERROR_NULL;
						break;
				}
			}
			s->csr = RS485_STA_RX_OK;
			break;              //��һ������
		}
		case RS485_STA_RX_OK: {
			switch(s->enRs485CmdType) {
				case RS485_CMD_READ_REGS:
					Sci_ReadRegs_ACK(s);
					break;
				case RS485_CMD_WRITE_REGS:
					Sci_WrReg_s_Decode_ACK(s);
					break;
				default:
					break;
			}
			USART1->CR1 |= (1<<3);
			g_u8SCI1TxEnable = 1;
		}
		//transmit complete, to switch receive wait 20ms
		case RS485_STA_TX_COMPLETE: {
			if(g_u8SCI1TxFinishFlag) {
				s->csr = RS485_STA_IDLE;
				s->u16Buffer[0] = 0;
				s->u16Buffer[1] = 0;
				s->u16Buffer[2] = 0;
				s->u16Buffer[3] = 0;
				g_u8SCI1TxFinishFlag = 0;
				s->ptr_no = 0;

				//RCSTA1bits.CREN = 1;    //ʹ�ܽ���
				//RC1IE = 1;// ʹ��EUSART2 �����ж�
				USART1->CR1 |= (1<<2);
				USART1->CR1 |= (1<<5);	
				//TXSTA1bits.TXEN = 0;    //��ֹ���ͣ����ܽ�ֹ������ͨ�ź����׶�
				g_u8SCI1TxEnable = 0;
			}
			break;
		}
		
		default: {
			s->csr = RS485_STA_IDLE;
			break;
		}
	}

	Sci1_CommonUpper_Tx_Deal(s);
	//Sci1_FaultChk();
}			

#endif

#if (defined _COMMOM_UPPER_SCI2)

//���ڳ�ʼ��������ò�Ʋ�������IO��
void InitUSART2_CommonUpper(void) {
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//RCC->AHBENR |= 1<<17;										//����GPIOA������ʱ��

	//Enable the USART2 Interrupt(ʹ��USART2�ж�)
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;   	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 


    //USART2_TX -> PA9 , USART2_RX -> PA3
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1);		//030��AF����ڷ�reg��datasheet��
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1);                                     
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	//���ڳ�ʼ��
    USART_InitStructure.USART_BaudRate = 19200;//���ô��ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��������λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//����ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����Ч��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//����������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//���ù���ģʽ
    USART_Init(USART2, &USART_InitStructure); //������ṹ��

	USART2->CR3 |= 1<<0;	//EIE����֡�����жϣ�ͬʱ���������ж�
	USART2->CR3 |= 1<<11;	//δ��ʹ��ǰ��д����ֹ�����ж�	

    USART_Cmd(USART2, ENABLE);//ʹ�ܴ���1
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);		   //ʹ�ܽ����ж�

    Sci_DataInit(&g_stSCI2CurrentMsgPtr);
}

void Sci2_CommonUpper_FaultChk(void) {
	UINT8 FaultCnt = 0;

	if(USART2->ISR&0x08) {		//�����������RXNEIE��EIEʹ�ܲ����жϣ���
		USART2->ICR |= 1<<3;	//���
		FaultCnt++;
	}

	if(USART2->ISR&0x04) {		//��⵽������Ĭ�Ͽ��������Ļ�CR3��ONEBIT��1������
								//USART_CR3��EIEʹ���ж�
		USART2->ICR |= 1<<2;	//���
		FaultCnt++;

	}

	if(USART2->ISR&0x02) {		//֡����USART_CR3��EIEʹ���жϣ���
		USART2->ICR |= 1<<1;	//���
		FaultCnt++;
	}

	if(USART2->ISR&0x01) {		//У������־ USART_CR1��PEIEʹ�ܸ��жϣ�����
		USART2->ICR |= 1<<0;	//���
		FaultCnt++;
	}

	if(FaultCnt) {
		g_u16SCI2CommuErrCnt++;
	}
}


void Sci2_CommonUpper_Rx_Deal(struct RS485MSG *s ) {
	USART2->CR1 &= ~(1<<5);		//�������Ǿ仰��ѡһ
	s->u16Buffer[s->ptr_no] = USART2->RDR; 				// ��RXFIFO �ж�ȡ���յ�������
	if((s->ptr_no == 0)&&(s->u16Buffer[0] != RS485_SLAVE_ADDR)&&(s->u16Buffer[0] != RS485_BROADCAST_ADDR)) {
		s->ptr_no = 0;
		s->u16Buffer[0] = 0;
	}
	else {
		if(s->ptr_no == 1) {
			switch(s->u16Buffer[s->ptr_no]) {
				case RS485_CMD_READ_REGS:
					s->enRs485CmdType = RS485_CMD_READ_REGS;
					break;
				/*
				case RS485_CMD_WRITE_REG:
					s->enRs485CmdType = RS485_CMD_WRITE_REG;
					break;
				*/
				case RS485_CMD_WRITE_REGS:
					s->enRs485CmdType = RS485_CMD_WRITE_REGS;
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
				case RS485_CMD_READ_REGS:
				//case RS485_CMD_WRITE_REG:
					if (s->ptr_no == 7) {	//	receive complete
						s->csr = RS485_STA_RX_COMPLETE;
						USART2->CR1 &= ~(1<<2);
						USART2->CR1 &= ~(1<<5);
					}
					break;
				case RS485_CMD_WRITE_REGS:
					if(s->ptr_no >= 7) {
						if((s->u16Buffer[6] != 0 && s->ptr_no == (s->u16Buffer[6] + 8))
							||(s->u16Buffer[6] == 0 && s->ptr_no == s->u16Buffer[5]+(s->u16Buffer[4]<<8)+8)) {
							s->csr = RS485_STA_RX_COMPLETE;
							USART1->CR1 &= ~(1<<2);
							USART1->CR1 &= ~(1<<5);
						}
					}
					break;
				default:
					//s->ptr_no = 0;
					//s->u16Buffer[0] = 0;
					break;
			}
		}
		s->ptr_no++;
		if(s->ptr_no == RS485_MAX_BUFFER_SIZE) {
			s->ptr_no = 0;
			s->u16Buffer[0] = 0;
		}
	}
	USART2->CR1 |= (1<<5);
}


void Sci2_CommonUpper_Tx_Deal(struct RS485MSG *s ) {
	if(0 == g_u8SCI2TxEnable) {
		return;
	}
	
	if(g_u16SCI2CommuErrCnt) {	//������żУ�����ȴ���Ҳ�ð�����ȫ�������꣬Ȼ�󲻻ظ�
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8SCI2TxFinishFlag = 1;
		g_u8SCI2TxEnable = 0;
		g_u16SCI2CommuErrCnt = 0;
		return;
	}

	while(!((USART2->ISR)&(1<<7)));				// 1<<6 Ҳ����
	if( s->ptr_no < s -> AckLenth) {
        USART2->TDR = s->u16Buffer[s->ptr_no];	//load data
		s->ptr_no++;
	}
	else {
		s->ptr_no = 0;
		s->csr = RS485_STA_TX_COMPLETE;
		g_u8SCI2TxFinishFlag = 1;
		g_u8SCI2TxEnable = 0;
		
		if(u8FlagUdFinishE2PROM) {
			u8FlagUdFinishE2PROM = 0;
			u8FlagUdFinish = 1;
		}
	}
}


void App_CommonUpper_Sci2(struct RS485MSG *s) {
	switch(s->csr) {
		//IDLE-����̬������50ms��ʹ�ܽ��գ�����㣩receive set
		case RS485_STA_IDLE: {
			break;
		}
		//receive complete, to deal the receive data
		case RS485_STA_RX_COMPLETE: {
			USART2->CR1 &= ~(1<<5);			//��ֹ�����ж�
			CRC_verify(s);
			if(s ->AckType == RS485_ACK_POS) {
				switch(s->enRs485CmdType) {
					case RS485_CMD_READ_REGS:
						Sci_ReadRegsDecode(s);
						break;
					case RS485_CMD_WRITE_REGS:
						Sci_WrRegsDecode(s);
						//s->csr = RS485_STA_RX_OK;
						break;
					default:
						s ->u16RdRegByteNum = 0;
						s ->AckType = RS485_ACK_NEG;
						s ->ErrorType = RS485_ERROR_NULL;
						break;
				}
			}
			s->csr = RS485_STA_RX_OK;		//receive the correct data, switch to transmit wait 50ms
			break;              			//��һ������
		}
		//receive ok, to transmit wait 50ms
		case RS485_STA_RX_OK: {
			switch(s->enRs485CmdType) {
				case RS485_CMD_READ_REGS:
					Sci_ReadRegs_ACK(s);
					break;
				case RS485_CMD_WRITE_REGS:
					Sci_WrReg_s_Decode_ACK(s);
					break;
				default:				//���defualt���üӴ������
					break;
			}
			USART2->CR1 |= (1<<3); 		//ʹ�ܷ���
			g_u8SCI2TxEnable = 1;
		}
		//transmit complete, to switch receive wait 20ms
		case RS485_STA_TX_COMPLETE: {
			if(g_u8SCI2TxFinishFlag) {
				s->csr = RS485_STA_IDLE;
				s->u16Buffer[0] = 0;
				s->u16Buffer[1] = 0;
				s->u16Buffer[2] = 0;
				s->u16Buffer[3] = 0;
				g_u8SCI2TxFinishFlag = 0;
				s->ptr_no = 0;
				USART2->CR1 |= (1<<2);	//ʹ�ܽ���
				USART2->CR1 |= (1<<5);	//ʹ��EUSART�����ж�
				//TXSTA1bits.TXEN = 0;    //��ֹ���ͣ����ܽ�ֹ������ͨ�ź����׶�
				g_u8SCI2TxEnable = 0;
			}
			break;
		}
		
		default: {
			s->csr = RS485_STA_IDLE;
			break;
		}
	}

	Sci2_CommonUpper_Tx_Deal(s);
	//Sci1_FaultChk();
}

#endif


void USART1_IRQHandler(void) {
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		#ifdef _COMMOM_UPPER_SCI1
		Sci1_CommonUpper_FaultChk();
		Sci1_CommonUpper_Rx_Deal(&g_stSCI1CurrentMsgPtr);
		#endif
	}
}


void USART2_IRQHandler(void) {
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		#ifdef _COMMOM_UPPER_SCI2
		Sci2_CommonUpper_FaultChk();
		Sci2_CommonUpper_Rx_Deal(&g_stSCI2CurrentMsgPtr);
		#endif
	}
}

