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
1，如何将中断向量表的寻找位置从0x0800 0000修改到0x0800 3000(假设为APP的地址)？
	A，修改寄存器VTOR的值(M3,M4可以)
	B，内存重映射(M0只能使用这个)

2，关于中断向量表的问题，假设M0和M3内核的启动方式是一样的，IAP的中断和APP的中断将不会重叠，不会错乱
	A，APP复位后，IAP启动，0x08000004复位中断向量开始
	B，IAP的中断将在0x08000004往后的中断向量表中选择，再跳到自己的IAP_main函数
	C，然后IAP的main函数跳到 0x08000004+N(自己定义的APP起始地址 + 4)，再跳到APP_main函数
	D，APP_main的中断由于内存映射，到0x20000000开始选择，与IAP_main函数产生中断不在同一个地方
	E，这样的话，就算都有同样中断，这是互不干扰的，复位过去也不需要过多关注SRAM中的中断
	F，复位应该不会重置SRAM中的东西(没找到相关支持)，就算重置也没关系，APP初始化会再次映射。
	G，不对，复位后所有寄存器都回reset值，所以不会再有重映射的问题，main flash重新映射回0x00000000则中断无冲突
3，看门狗的问题，LSI在主APP开了，就不能关掉，跳到IAP会有问题吗
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
		InitSystemWakeUp();		//少了这个初始化函数导致写出错，miss
        while(1) {
			App_SysTime();
			App_UpgrateFaultMonitor();
			App_FlashUpgrate();
            App_UpdateFinishChk();      // 升级结束判断
        }
    }
	return 0;
}


void InitIO(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);			//开启GPIOA的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);			//开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);			//开启GPIOC的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE); 		//开启GPIOF的外设时钟

	//PA1_MCUO_AFE_VPRO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//PB1_PWSV_STB，PB2_LED1，PB15_PWSV_CTR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		//选择要用的GPIO引脚	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	//设置引脚模式为上拉输入模式				 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


void InitSystemWakeUp(void) {
	MCUO_PWSV_STB = 0;
	
	// MCUO_PWSV_LDO = 1;
	MCUO_PWSV_CTR = 1;
}

//使用reset不需要deinit相关寄存器
void IAP_To_APP_Jump(void) {
	if (((*(__IO uint32_t *)FLASH_ADDR_APP_START) & 0x2FFE0000) == 0x20000000) {	//判断APP区是否有代码
		//使用reset不需要了
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
		__delay_ms(10);				//没这个延时reset的时候上位机报错，导致后续不行
		u8FlagUdFinish = 0;
		MCU_RESET();
	}
}

void InitDelay(void) {
	SysTick->CTRL &= ~(1<<2);		//使用外部时钟
	//fac_us=SystemCoreClock/8000000;	//为系统时钟的1/8 
	fac_us=6;
	fac_ms=(INT16)fac_us*1000;//每个ms需要的systick时钟数   

}


void __delay_ms(INT16 ms) {	 		  	  
	INT32 temp;		   
	SysTick->LOAD=(INT32)ms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
	do {
		temp=SysTick->CTRL;
	} while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器
}

//030所有时钟均为向上计数
void InitTimer(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);		//时钟3使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	//定时器初始化
	TIM_TimeBaseStructure.TIM_Period = 499; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000 - 1; 					//设置用来作为TIMx时钟频率除数的预分频值——计数分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//设置时钟分割:TDTS = Tck_tim——时钟分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure); 			//根据指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM17,TIM_IT_Update,ENABLE ); 					//使能指定中断,允许更新中断

	/*中断嵌套设计*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;	//抢占优先级0级，没响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM17, ENABLE);  //使能TIMx
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

	if(s_u16Cnt10ms != g_u810msClockCnt) {		//10ms定时标志
		s_u16Cnt10ms = g_u810msClockCnt;
		switch(g_u810msClockCnt) {
			case 0:
				s_u16Cnt200ms++;
				break;
			
			default:
				break;
		}
	}

	g_st_SysTimeFlag.bits.b1Sys200msFlag = 0;			 //屏蔽掉能使通讯异常
	if(s_u16Cnt200ms >= 20) {
		s_u16Cnt200ms = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag = 1;		 //200ms定时标志
	}

	if(1 == g_st_SysTimeFlag.bits.b1Sys200msFlag) {
		++u8LEDcnt;
		if(u8LEDcnt>=5) {
			MCUO_DEBUG_LED1 = !MCUO_DEBUG_LED1;
			u8LEDcnt = 0;
		}
    }
}


//定时器3中断服务程序
void TIM17_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET) {  //检查TIM3更新中断发生与否
		
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);  //清除TIMx更新中断标志
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

