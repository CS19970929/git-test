#include "main.h"

void App_FlashUpgrate(void);
void App_UpdateFinishChk(void);
void IAP_To_APP_Jump(void);
void InitDelay(void);
void InitIO(void);
void InitSystemWakeUp(void);
void InitTimer(void);
void App_SysTime(void);
void App_UpgrateFaultMonitor(void);
void InitUSART_CommonUpper(void);

volatile union SYS_TIME g_st_SysTimeFlag;

typedef void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint8_t u8FlagUdFinishE2PROM = 0;
uint8_t u8FlagUdFinish = 0;

static INT8  fac_us=0;//us
static INT16 fac_ms=0;//ms

UINT8 g_u810msClockCnt=0;



/*
1����ν��ж��������Ѱ��λ�ô�0x0800 0000�޸ĵ�0x0800 3000(����ΪAPP�ĵ�ַ)��
	A���޸ļĴ���VTOR��ֵ(M3,M4����)
	B���ڴ���ӳ��(M0ֻ��ʹ�����)

2�������ж�����������⣬����M0��M3�ں˵�������ʽ��һ���ģ�IAP���жϺ�APP���жϽ������ص����������
	A��APP��λ��IAP������0x08000004��λ�ж�������ʼ
	B��IAP���жϽ���0x08000004������ж���������ѡ���������Լ���IAP_main����
	C��Ȼ��IAP��main�������� 0x08000004+N(�Լ������APP��ʼ��ַ + 4)��������APP_main����
	D��APP_main���ж������ڴ�ӳ�䣬��0x20000000��ʼѡ����IAP_main���������жϲ���ͬһ���ط�
	E�������Ļ������㶼��ͬ���жϣ����ǻ������ŵģ���λ��ȥҲ����Ҫ�����עSRAM�е��ж�
	F����λӦ�ò�������SRAM�еĶ���(û�ҵ����֧��)����������Ҳû��ϵ��APP��ʼ�����ٴ�ӳ�䡣
	G�����ԣ���λ�����мĴ�������resetֵ�����Բ���������ӳ������⣬main flash����ӳ���0x00000000���ж��޳�ͻ
3�����Ź������⣬LSI����APP���ˣ��Ͳ��ܹص�������IAP����������
*/
int main(void) {
	SystemInit();
	if(FlashReadOneHalfWord(FLASH_ADDR_UPDATE_FLAG) == FLASH_TO_APP_VALUE) {
		IAP_To_APP_Jump();
	}
    else {
		InitIO();
		InitDelay();
		InitUSART_CommonUpper();
		InitTimer();
		InitSystemWakeUp();		//���������ʼ����������д����miss
        while(1) {
			App_SysTime();
			App_UpgrateFaultMonitor();
			App_FlashUpgrate();
            App_UpdateFinishChk();      // ���������ж�
        }
    }
	return 0;
}


void InitIO(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);			//����GPIOA������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);			//����GPIOB������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);			//����GPIOC������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE); 		//����GPIOF������ʱ��

	//PA1_MCUO_AFE_VPRO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//PB1_PWSV_STB��PB2_LED1��PB15_PWSV_CTR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		//ѡ��Ҫ�õ�GPIO����	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	//��������ģʽΪ��������ģʽ				 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


void InitSystemWakeUp(void) {
	MCUO_PWSV_STB = 0;
	
	// MCUO_PWSV_LDO = 1;
	MCUO_PWSV_CTR = 1;
}

//ʹ��reset����Ҫdeinit��ؼĴ���
void IAP_To_APP_Jump(void) {
	if (((*(__IO uint32_t *)FLASH_ADDR_APP_START) & 0x2FFE0000) == 0x20000000) {	//�ж�APP���Ƿ��д���
		//ʹ��reset����Ҫ��
		/*
		GPIO_DeInit(GPIOB);					//DeInit GPIO 
		I2C_Cmd(I2C1, DISABLE);
		I2C_DeInit(I2C1);
		*/
		JumpAddress = *(__IO uint32_t *)(FLASH_ADDR_APP_START + 4);				//Jump to user application
		Jump_To_Application = (pFunction)JumpAddress;
		__set_MSP(*(__IO uint32_t *)FLASH_ADDR_APP_START);							//Initialize user application's Stack Pointer
		Jump_To_Application();													//Jump to application
	}
}


void App_UpgrateFaultMonitor(void) {
	static UINT8 s_u8UpgrateFlag = 0;
	static UINT16 s_u16UpgrateTCnt = 0;
	if(!u8FlashReceiveCnt || 0 == g_st_SysTimeFlag.bits.b1Sys200msFlag) {
		return;
	}
	if(s_u8UpgrateFlag != u8FlashReceiveCnt) {
		s_u8UpgrateFlag = u8FlashReceiveCnt;
		s_u16UpgrateTCnt = 0;
	}
	else {
		if(++s_u16UpgrateTCnt > UPGRATER_TIMEOUT) {
			s_u16UpgrateTCnt = 0;
			s_u8UpgrateFlag = 0;
			u8FlashReceiveCnt = 0;
		}
	}
}

void App_FlashUpgrate(void) {
	#ifdef _COMMOM_UPPER_SCI1
	App_CommonUpper_Sci1(&g_stSCI1CurrentMsgPtr);
	#endif

	#ifdef _COMMOM_UPPER_SCI2
	App_CommonUpper_Sci2(&g_stSCI2CurrentMsgPtr);
	#endif
}


void App_UpdateFinishChk(void) {
	if(1 == u8FlagUdFinish) {
		__delay_ms(10);				//û�����ʱreset��ʱ����λ���������º�������
		u8FlagUdFinish = 0;
		MCU_RESET();
	}
}

void InitDelay(void) {
	SysTick->CTRL &= ~(1<<2);		//ʹ���ⲿʱ��
	//fac_us=SystemCoreClock/8000000;	//Ϊϵͳʱ�ӵ�1/8 
	fac_us=6;
	fac_ms=(INT16)fac_us*1000;//ÿ��ms��Ҫ��systickʱ����   

}


void __delay_ms(INT16 ms) {	 		  	  
	INT32 temp;		   
	SysTick->LOAD=(INT32)ms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����  
	do {
		temp=SysTick->CTRL;
	} while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����
}

//030����ʱ�Ӿ�Ϊ���ϼ���
void InitTimer(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);		//ʱ��3ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	//��ʱ����ʼ��
	TIM_TimeBaseStructure.TIM_Period = 499; 					//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000 - 1; 					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ����������Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//����ʱ�ӷָ�:TDTS = Tck_tim����ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure); 			//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_ITConfig(TIM17,TIM_IT_Update,ENABLE ); 					//ʹ��ָ���ж�,��������ж�

	/*�ж�Ƕ�����*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;	//��ռ���ȼ�0����û��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM17, ENABLE);  //ʹ��TIMx
}


void InitUSART_CommonUpper(void) {
	#ifdef _COMMOM_UPPER_SCI1
	InitUSART1_CommonUpper();
	#endif
	
	#ifdef _COMMOM_UPPER_SCI2
	InitUSART2_CommonUpper();
	#endif
}

void App_SysTime(void) {
	static UINT8 u8LEDcnt = 0;
	static UINT16 s_u16Cnt10ms = 0;
	static UINT16 s_u16Cnt200ms = 0;

	if(s_u16Cnt10ms != g_u810msClockCnt) {		//10ms��ʱ��־
		s_u16Cnt10ms = g_u810msClockCnt;
		switch(g_u810msClockCnt) {
			case 0:
				s_u16Cnt200ms++;
				break;
			
			default:
				break;
		}
	}

	g_st_SysTimeFlag.bits.b1Sys200msFlag = 0;			 //���ε���ʹͨѶ�쳣
	if(s_u16Cnt200ms >= 20) {
		s_u16Cnt200ms = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag = 1;		 //200ms��ʱ��־
	}

	if(1 == g_st_SysTimeFlag.bits.b1Sys200msFlag) {
		++u8LEDcnt;
		if(u8LEDcnt>=5) {
			MCUO_DEBUG_LED1 = !MCUO_DEBUG_LED1;
			u8LEDcnt = 0;
		}
    }
}


//��ʱ��3�жϷ������
void TIM17_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET) {  //���TIM3�����жϷ������
		
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);  //���TIMx�����жϱ�־
		#if 0
		if((++g_u81msCnt) >= 2)                             //1ms
        {
            g_u81msCnt = 0;
            g_u81msClockCnt ++;

            if(g_u81msClockCnt >= 2)                       //2ms
            {
                g_u81msClockCnt = 0;
                g_u810msClockCnt++;
                if(g_u810msClockCnt >= 5)                  //10ms
                {
                    g_u810msClockCnt = 0;
                }
            }
        }
		#endif
		
        if(++g_u810msClockCnt >= 20) {                 //10ms
            g_u810msClockCnt = 0;
        }
	}
}

