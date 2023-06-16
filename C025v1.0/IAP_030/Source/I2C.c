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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE); 			//开启I2C2外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);				//开启GPIOF的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);		  		//开启GPIOB的外设时钟
	
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;		//0: 模拟噪声滤波器开启
	I2C_InitStruct.I2C_DigitalFilter = 0x00;						//数字滤波器关闭
	I2C_InitStruct.I2C_OwnAddress1 =0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	
	I2C_InitStruct.I2C_Timing = 0x30E3363D;							//妈的，主频变了，这里忘了改，还能时行时不行？搞得以为我代码没错，时序没错。
	/* I2C Peripheral Enable */										//时序继续错！上升下降沿时间有需求，EEPROM貌似要求20-50ns(综合取并集)？现改为40ns，还有吗？？
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
    //CLRWDT();                           //看门狗定时器清零，ST的后面加
	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//等待总线不忙
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	//__delay_ms(5);
	//sEE_WaitEepromStandbyState();			//写必须要这个，读可以不用，但为了保险起见，加上，后续观察这个函数的耗时，对别的通讯动作，时序影响
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);	//I2C_Reload_Mode，传完两个地址能继续传

	/* Send MSB of memory address */
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR寄存器为空置1，跳出循环，则写
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
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TCR) == RESET) {		//等待发送完成标志，由硬件清0，I2C_Reload_Mode才会产生TC，但是不会带stop信号，能继续传
		if((sEETimeout--) == 0) {
			sEE_I2CFaultcnt = sEE_I2CFaultcnt + 3;
			return sEE_TIMEOUT_UserCallback();
		}
	}
	/* Update CR2 : set Slave Address , set write request, generate Start and set end mode */
	//不产生起始或者开始信号，在Reload_Mode为1时，Start也没用，也即不会产生起始信号和发送地址，只是更新某些功能
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);	//自动结束，产生stop信号

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//出现在发送中断，则发
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
	
	__delay_ms(5);								//到处想，直接这里一加完美了，巧妙得一匹
	return sEE_OK;
	
}


//后续维护人员禁止使用这个函数
//这个地方，不能return sEE_TIMEOUT_UserCallback()的值！会出错！
uint8_t EEPROM_Read(uint16_t addr) {

    uint8_t u8tmp = 0xff;

	/*
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_FLAG_BUSY) != RESET) {	//等待总线不忙
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/

	//sEE_WaitEepromStandbyState();
	//I2C_SoftEnd_Mode，是在地址数据发送以后产生一个restart信号，以表示用来读取E2的数据
	I2C_TransferHandling(sEE_I2C, sEEAddress, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TXIS) == RESET) {		//I2Cx_TXDR寄存器为空置1，跳出循环，则写
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


	/*	//去掉了反而变好了.....哪里return点哪里，无语了
	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_TC) == RESET) {		//等待发送完成标志，由硬件清0
		if((sEETimeout--) == 0) {
			return sEE_TIMEOUT_UserCallback();
		}
	}
	*/
	/* Update CR2 : set Slave Address , set read request, generate Start and set end mode */
	//I2C_AutoEnd_Mode，NBYTES 个数据传输完后，会自动发送一个停止条件。
	//I2C_Generate_Start_Read应该会再次发送读地址 0xA0 + 0x01 = 0xA1
	I2C_TransferHandling(sEE_I2C, sEEAddress, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	

	sEETimeout = sEE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(sEE_I2C, I2C_ISR_RXNE) == RESET) {		//收到非空，则读
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

    return u8tmp;         //返回EEDATA存储的数据
}




//这个和EEPROM_Read()的返回值类型不能搞错！
//这个函数在出现静电搞错的时候，运行下面一大堆会不会又出现超时错误？可以复现，先留着
uint16_t ReadEEPROM_Word_NoZone(uint16_t addr) {

	uint16_t tmp16a;
	uint8_t  tmp8a,tmp8b;	
	
	tmp8a = EEPROM_Read(addr);		                        //读取低位地址A对应的数据
	tmp8b = EEPROM_Read(addr+1);	                        //读取高位地址A+1对应的数据
	tmp16a = tmp8b;
	tmp16a = (tmp16a<<8) | tmp8a;	                        //数据存储

	return tmp16a;

}


//主要调这个，加了几句话
int8_t WriteEEPROM_Word_NoZone(uint16_t                 addr, uint16_t data)
{
    uint8_t  tmp8a;
    uint8_t  WriteCounter = 0;		            /* EEPROM写计数----*/
    uint16_t tmp_addr;
    uint16_t tmp16;
	int8_t result = 0;
	tmp_addr = addr;						//移植忘了这句话

	//MCUO_E2PR_WP = 0;

	WriteCounter = 0;
	do {
		tmp8a = data & 0xff;				//获取数据的低8位
		EEPROM_Write(tmp_addr, tmp8a);		//数据的低8位写入EEPROM
		tmp8a = data >> 8;					//获取数据的高8位
		//__delay_ms(5);
		EEPROM_Write(tmp_addr+1, tmp8a);	//数据的高8位写入EEPROM
	
		//读取写入的数据
		sEE_WaitEepromStandbyState();
		tmp8a = EEPROM_Read(tmp_addr);		//获取刚存入EEPROM的低8位数据
		tmp16 = tmp8a;
		tmp8a = EEPROM_Read(tmp_addr+1);	//获取刚存入EEPROM的高8位数据
		tmp16 = (tmp8a<<8) |tmp16 ; 		//存储读到的数据于变量tmp16
	
		WriteCounter++;
		if(WriteCounter > 2) {				/*判断tmp16 != data的计数*/
			result = -1;
			++sEE_I2CFaultcnt;
			break;
		}
	}while(tmp16 != data);
	//__delay_ms(5);						//防止没写完就失效，另外那个就不用，因为前面已经read了
	//MCUO_E2PR_WP = 1;	

	return result;
}

#endif

