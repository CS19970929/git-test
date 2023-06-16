#include "main.h"

#if 1
__IO uint16_t  sEEAddress = 0;   
__IO uint32_t  sEETimeout = sEE_LONG_TIMEOUT;
__IO uint16_t  sEEDataNum;
uint8_t Faultcnt = 0;
uint8_t g_u8UpdateFlag = 0;
uint8_t sEE_I2CFaultcnt = 0;

#ifdef __EEPROM_TEST
UINT16 a = 0;
UINT16 b = 0;
UINT16 c = 0;
UINT16 d = 0;
#endif

void I2C_Configuration(void) {
	
	I2C_InitTypeDef  I2C_InitStruct;
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* I2C configuration */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE); 			//����I2C2����ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);				//����GPIOF������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);		  		//����GPIOB������ʱ��
	
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;		//0: ģ�������˲�������
	I2C_InitStruct.I2C_DigitalFilter = 0x00;						//�����˲����ر�
	I2C_InitStruct.I2C_OwnAddress1 =0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	
	I2C_InitStruct.I2C_Timing = 0x30E3363D;							//��ģ���Ƶ���ˣ��������˸ģ�����ʱ��ʱ���У������Ϊ�Ҵ���û��ʱ��û��
	/* I2C Peripheral Enable */										//ʱ������������½���ʱ��������EEPROMò��Ҫ��20-50ns(�ۺ�ȡ����)���ָ�Ϊ40ns�������𣿣�
	I2C_Cmd(sEE_I2C, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(sEE_I2C, &I2C_InitStruct);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOF , &GPIO_InitStructure);
  
}


void I2C_EE_Init(void) {

  I2C_Configuration();
  sEEAddress = sEE_HW_ADDRESS;  
}

#ifdef __EEPROM_TEST
void EEPROM_test(void) {
	WriteEEPROM_Word_NoZone(0x1234, 1124);
	WriteEEPROM_Word_NoZone(0x1236,1124);
	WriteEEPROM_Word_NoZone(0x1238,1124);
	a = ReadEEPROM_Word_NoZone(0x1234);
	b = ReadEEPROM_Word_NoZone(0x1236);
	c = ReadEEPROM_Word_NoZone(0x1238);

	WriteEEPROM_Word_NoZone(EEPROM_FLASHUPDATE_ADDR, EEPROM_FLASHUPDATE_VALUE);
	d = ReadEEPROM_Word_NoZone(EEPROM_FLASHUPDATE_ADDR);
}
#endif

uint32_t sEE_TIMEOUT_UserCallback(void) {
  /* Block communication and all processes */

	++Faultcnt;
	return 0;

}

uint32_t sEE_WaitEepromStandbyState(void)      
{
  __IO uint32_t sEETrials = 0;
  
  /* Keep looping till the slave acknowledge his address or maximum number 
  of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER define
  in stm32373c_eval_i2c_ee.h file) */
  
  /* Configure CR2 register : set Slave Address and end mode */
  I2C_TransferHandling(sEE_I2C, sEEAddress, 0, I2C_AutoEnd_Mode, I2C_No_StartStop);  
  
  do
  { 
    /* Initialize sEETimeout */
    sEETimeout = sEE_FLAG_TIMEOUT;
    
    /* Clear NACKF */
    I2C_ClearFlag(sEE_I2C, I2C_ICR_NACKCF | I2C_ICR_STOPCF);
    
    /* Generate start */
    I2C_GenerateSTART(sEE_I2C, ENABLE);
    
    /* Wait until timeout elapsed */
    while (sEETimeout-- != 0); 
    
    /* Check if the maximum allowed numbe of trials has bee reached */
    if (sEETrials++ == sEE_MAX_TRIALS_NUMBER)
    {
      /* If the maximum number of trials has been reached, exit the function */
      return sEE_TIMEOUT_UserCallback();
    }
  }
  while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_NACKF) != RESET);
  
  /* Clear STOPF */
  I2C_ClearFlag(sEE_I2C, I2C_ICR_STOPCF);
  
  /* Return sEE_OK if device is ready */
  return sEE_OK;
}


int8_t EEPROM_Write(uint16_t addr, uint8_t val) {
    //CLRWDT();                           //���Ź���ʱ�����㣬ST�ĺ����
	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//�ȴ����߲�æ
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	//__delay_ms(5);
	//sEE_WaitEepromStandbyState();			//д����Ҫ����������Բ��ã���Ϊ�˱�����������ϣ������۲���������ĺ�ʱ���Ա��ͨѶ������ʱ��Ӱ��
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);	//I2C_Reload_Mode������������ַ�ܼ�����

	/* Send MSB of memory address */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR�Ĵ���Ϊ����1������ѭ������д
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 1;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_SendData(sEE_I2C, (uint8_t)((addr & 0xFF00) >> 8));  


	/* Send LSB of memory address  */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 2;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_SendData(sEE_I2C, (uint8_t)(addr & 0x00FF));

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TCR) == RESET) {		//�ȴ�������ɱ�־����Ӳ����0��I2C_Reload_Mode�Ż����TC�����ǲ����stop�źţ��ܼ�����
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 3;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	/* Update CR2 : set Slave Address , set write request, generate Start and set end mode */
	//��������ʼ���߿�ʼ�źţ���Reload_ModeΪ1ʱ��StartҲû�ã�Ҳ�����������ʼ�źźͷ��͵�ַ��ֻ�Ǹ���ĳЩ����
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);	//�Զ�����������stop�ź�

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//�����ڷ����жϣ���
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 4;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_SendData(sEE_I2C, val);
		
	/* Wait until STOPF flag is set */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_STOPF) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 5;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	I2C_ClearFlag(sEE_I2C, I2C_ICR_STOPCF);
	
	__delay_ms(5);								//�����룬ֱ������һ�������ˣ������һƥ
	return sEE_OK;
	
}


//����ά����Ա��ֹʹ���������
//����ط�������return sEE_TIMEOUT_UserCallback()��ֵ�������
uint8_t EEPROM_Read(uint16_t addr) {

    uint8_t u8tmp = 0xff;

	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//�ȴ����߲�æ
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/

	//sEE_WaitEepromStandbyState();
	//I2C_SoftEnd_Mode�����ڵ�ַ���ݷ����Ժ����һ��restart�źţ��Ա�ʾ������ȡE2������
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR�Ĵ���Ϊ����1������ѭ������д
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 6;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	I2C_SendData(sEE_I2C, (uint8_t)((addr & 0xFF00) >> 8));


	sEETimeout = sEE_LONG_TIMEOUT;  
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 7;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	I2C_SendData(sEE_I2C, (uint8_t)(addr & 0x00FF));


	/*	//ȥ���˷��������.....����return�����������
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TC) == RESET) {		//�ȴ�������ɱ�־����Ӳ����0
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	/* Update CR2 : set Slave Address , set read request, generate Start and set end mode */
	//I2C_AutoEnd_Mode��NBYTES �����ݴ�����󣬻��Զ�����һ��ֹͣ������
	//I2C_Generate_Start_ReadӦ�û��ٴη��Ͷ���ַ 0xA0 + 0x01 = 0xA1
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_RXNE) == RESET) {		//�յ��ǿգ����
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 8;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	u8tmp= I2C_ReceiveData(sEE_I2C);
	
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_STOPF) == RESET) {
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 9;
			sEE_TIMEOUT_UserCallback();
			return 0;
		}
	}
	I2C_ClearFlag(sEE_I2C, I2C_ICR_STOPCF);

    return u8tmp;         //����EEDATA�洢������
}




//�����EEPROM_Read()�ķ���ֵ���Ͳ��ܸ��
//��������ڳ��־������ʱ����������һ��ѻ᲻���ֳ��ֳ�ʱ���󣿿��Ը��֣�������
uint16_t ReadEEPROM_Word_NoZone(uint16_t addr) {

	uint16_t tmp16a;
	uint8_t  tmp8a,tmp8b;	
	
	tmp8a = EEPROM_Read(addr);		                        //��ȡ��λ��ַA��Ӧ������
	tmp8b = EEPROM_Read(addr+1);	                        //��ȡ��λ��ַA+1��Ӧ������
	tmp16a = tmp8b;
	tmp16a = (tmp16a<<8) | tmp8a;	                        //���ݴ洢

	return tmp16a;

}


//��Ҫ����������˼��仰
int8_t WriteEEPROM_Word_NoZone(uint16_t                 addr, uint16_t data)
{
    uint8_t  tmp8a;
    uint8_t  WriteCounter = 0;		            /* EEPROMд����----*/
    uint16_t tmp_addr;
    uint16_t tmp16;
	int8_t result = 0;
	tmp_addr = addr;						//��ֲ������仰

	//MCUO_E2PR_WP = 0;

	WriteCounter = 0;
	do {
		tmp8a = data & 0xff;				//��ȡ���ݵĵ�8λ
		EEPROM_Write(tmp_addr, tmp8a);		//���ݵĵ�8λд��EEPROM
		tmp8a = data >> 8;					//��ȡ���ݵĸ�8λ
		//__delay_ms(5);
		EEPROM_Write(tmp_addr+1, tmp8a);	//���ݵĸ�8λд��EEPROM
	
		//��ȡд�������
		sEE_WaitEepromStandbyState();
		tmp8a = EEPROM_Read(tmp_addr);		//��ȡ�մ���EEPROM�ĵ�8λ����
		tmp16 = tmp8a;
		tmp8a = EEPROM_Read(tmp_addr+1);	//��ȡ�մ���EEPROM�ĸ�8λ����
		tmp16 = (tmp8a<<8) |tmp16 ; 		//�洢�����������ڱ���tmp16
	
		WriteCounter++;
		if(WriteCounter > 2) {				/*�ж�tmp16 != data�ļ���*/
			result = -1;
			++sEE_I2CFaultcnt;
			break;
		}
	}while(tmp16 != data);
	//__delay_ms(5);						//��ֹûд���ʧЧ�������Ǹ��Ͳ��ã���Ϊǰ���Ѿ�read��
	//MCUO_E2PR_WP = 1;	

	return result;
}

#endif

