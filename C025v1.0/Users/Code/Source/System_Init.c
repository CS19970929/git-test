#include "main.h"

volatile union SYS_TIME g_st_SysTimeFlag;
struct CBC_ELEMENT CBC_Element;

static INT8  fac_us=0;	//us
static INT16 fac_ms=0;	//ms

UINT8 g_u81msCnt=0;
UINT8 g_u810msClockCnt=0;
UINT8 g_u81msClockCnt = 0;

UINT8 gu8_200msCnt = 0;
UINT8 gu8_200msAccClock_Flag = 0;

void IWDG_HaltConfig(void);


void InitDelay(void) {
	SysTick->CTRL &= ~(1<<2);				//使用外部时钟
	//这句话到底什么情况？？？？？害死我的系统时钟慢了十几倍
	//fac_us = SystemCoreClock / 8000000;		//SysTick时钟是SYSCLK 8分频，即SysTick时钟频率=SYSCLK/8，1us要计的个数为还得/1MHz
	//fac_us = 6;
	//fac_us = (UINT8)(SystemCoreClock / 8000000);	//我又改过来了，但是还是不行

	switch(SystemCoreClock) {
		case 8000000:
			fac_us = 1;
			fac_ms = (INT16)fac_us * 1000;	//每个ms需要的systick时钟数
			break;
		case 12000000:
			//fac_us = 1.5;
			fac_us = 1;						//us变得不准了，12M建议只能倍频
			fac_ms = (INT16)fac_us * 1500;	
			break;
		case 48000000:
			fac_us = 6;
			fac_ms = (INT16)fac_us * 1000;	//每个ms需要的systick时钟数
			break;
		default:
			break;
	}
}


void __delay_us(UINT32 nus) {		
	UINT32 temp;	    	 
	SysTick->LOAD = nus*fac_us; 					//时间加载	  		 
	SysTick->VAL = 0x00;        					//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL = 0X00;      					 //清空计数器	 
}


//这个是非中断方式的延时，倘若使用中断式延时，在中断中使用延时会出现中断嵌套问题，很容易出错
void __delay_ms(UINT16 ms) {	 		  	  
	UINT32 temp;		   
	SysTick->LOAD = (UINT32)ms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;           //清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;          //开始倒数
	do {
		temp=SysTick->CTRL;
	} while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL = 0X00;       //清空计数器
}


void InitIO(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);			//开启GPIOA的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);			//开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);			//开启GPIOC的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);			//开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);			//开启GPIOB的外设时钟
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

	//MCUO_DEBUG_LED1 = 1;
}


void InitTimer(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);		//时钟3使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	//定时器初始化
	TIM_TimeBaseStructure.TIM_Period = 500 - 1; 					//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000 - 1; 	//设置用来作为TIMx时钟频率除数的预分频值——计数分频
	//TIM_TimeBaseStructure.TIM_Prescaler = 1; 					//设置用来作为TIMx时钟频率除数的预分频值——计数分频
	//TIM_TimeBaseStructure.TIM_Prescaler = 16;
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


//使用LSI，38KHz
void Init_IWDG(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); 		//使能PWR外设时钟，待机模式，RTC，看门狗
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);	//打开独立看门狗寄存器操作权限
    IWDG_SetPrescaler(IWDG_Prescaler_64);			//预分频系数
    IWDG_SetReload(160);							//设置重载计数值，k = Xms / (1 / (40KHz/64)) = X/64*40; 4096最高
    												//800——1.28s，80——128ms
    IWDG_ReloadCounter();    						//喂狗
    IWDG_Enable();        							//使能IWDG
}


void InitCBC(NVIC_OnOFF NVIC_Status) {
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//InitIO()有了

	switch(NVIC_Status) {
		case NVIC_INIT:
			//PB12_CBC
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				//选择要用的GPIO引脚 
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//设置引脚模式为上拉输入模式
			GPIO_Init(GPIOB, &GPIO_InitStructure);					//调用库函数，初始化GPIO
			break;
	
		case NVIC_OPEN:
			//设置中断线12，EXTI12和PB12挂钩
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
			//配置外部上升沿中断
			EXTI_InitStruct.EXTI_Line = EXTI_Line12;				
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;	//下降沿触发
			EXTI_InitStruct.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStruct);
			
			//中断嵌套设计
		  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;		//使能外部中断通道
		  	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;		//抢占优先级0
		  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
		  	NVIC_Init(&NVIC_InitStructure);
			break;
			
		case NVIC_CLOSE:
			//设置中断线12，EXTI12和PB12挂钩
			//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
			//配置外部上升沿中断
			EXTI_InitStruct.EXTI_Line = EXTI_Line12;				
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;	//下降沿触发
			EXTI_InitStruct.EXTI_LineCmd = DISABLE;
			EXTI_Init(&EXTI_InitStruct);
			break;
			
		default:
			break;
	}	
}


void CBC_TimeCtrl_Relay(void) {
	static UINT16 su16_CBC_CHG_TCnt = 0;
	static UINT16 su16_CBC_DSG_TCnt = 0;
	static UINT16 su16_CBC_Recover_TCnt = 0;

	if(0 == g_st_SysTimeFlag.bits.b1Sys1000msFlag1) {
		return;
	}

	if(CBC_Element.u8CBC_CHG_ErrFlag) {
		//++CBC_Element.u8CBC_CHG_Cnt;				//不能放这里，不然BUG错了
		if(CBC_Element.u8CBC_CHG_Cnt >= 2) {		//第三次进来立刻关掉全部
			System_OnOFF_Func.bits.b1OnOFF_MOS_Relay = 0;
			ChargerLoad_Func.bits.b1OFFDriver_CBC = 1;
			CBC_Element.u8CBC_CHG_Cnt = 0;
			CBC_Element.u8CBC_CHG_ErrFlag = 0;
		}
		else {
			if(++su16_CBC_CHG_TCnt > 30) {			//这个不需要在下面复位，因为这个为0才有可能跳最后一个else判断
				su16_CBC_CHG_TCnt = 0;
				++CBC_Element.u8CBC_CHG_Cnt;		//打开，计一次
				CBC_Element.u8CBC_CHG_ErrFlag = 0;
			}
		}
		if(su16_CBC_Recover_TCnt)su16_CBC_Recover_TCnt = 0;
	}
	else if(CBC_Element.u8CBC_DSG_ErrFlag) {
		if(CBC_Element.u8CBC_DSG_Cnt >= 2) {
			System_OnOFF_Func.bits.b1OnOFF_MOS_Relay = 0;
			ChargerLoad_Func.bits.b1OFFDriver_CBC = 1;
			CBC_Element.u8CBC_DSG_Cnt = 0;
			CBC_Element.u8CBC_DSG_ErrFlag = 0;
		}
		else {
			if(++su16_CBC_DSG_TCnt > 30) {
				su16_CBC_DSG_TCnt = 0;
				++CBC_Element.u8CBC_DSG_Cnt;
				CBC_Element.u8CBC_DSG_ErrFlag = 0;
			}
		}
		if(su16_CBC_Recover_TCnt)su16_CBC_Recover_TCnt = 0;
	}
	else {
		if(CBC_Element.u8CBC_CHG_Cnt) {
			if(++su16_CBC_Recover_TCnt > 60) {
				su16_CBC_Recover_TCnt = 0;
				CBC_Element.u8CBC_CHG_Cnt = 0;
			}
		}
		else if(CBC_Element.u8CBC_DSG_Cnt) {
			if(++su16_CBC_Recover_TCnt > 60) {
				su16_CBC_Recover_TCnt = 0;
				CBC_Element.u8CBC_DSG_Cnt = 0;
			}
		}
		if(su16_CBC_CHG_TCnt)su16_CBC_CHG_TCnt = 0;
		if(su16_CBC_DSG_TCnt)su16_CBC_DSG_TCnt = 0;
	}
}


//有InitDAC()和InitPWM()函数相关
void CBC_Relay(void) {
	static UINT8 su8_CBCStartUp_Flag = 0;
	static UINT8 su8_CBCStatus_Flag = CLOSE;
	static UINT16 su16_CBCInitDelayCnt = 0;

	if(SystemStatus.bits.b1StartUpBMS) {		//意味着管子已经可以开始判断是否打开还是关闭
		return;									//接触器打开是否会出现干扰的问题？还是说要完全打开才打开这个中断？后续观察
	}

	if(0 == g_st_SysTimeFlag.bits.b1Sys10msFlag5) {
		return;
	}

	switch(su8_CBCStartUp_Flag) {
		case 0:
			InitCBC(NVIC_INIT);
			su8_CBCStartUp_Flag = 1;
			break;
			
		case 1:
			//因为只有一个可能打开，别的为CLOSE，为0值，或关系不受影响。放电管均为PE14，不会出现引脚被复用导致紊乱的问题
			//但是，如果别的地方调用，这个标志位虽然置位了，但是插进别的时序延迟打开，则有问题，所以直接观察寄存器是最佳选择
			//switch(SystemStatus.bits.b1Status_MOS_DSG | SystemStatus.bits.b1Status_Relay_DSG | SystemStatus.bits.b1Status_Relay_MAIN) {
			//switch(MCUO_MOS_DSG | MCUO_RELAY_DSG | MCUO_RELAY_MAIN) {
			switch(su8_CBCStatus_Flag) {		//果然，加个标志位还是简洁明了很多	
				case CLOSE:
					if(OPEN == (MCUO_MOS_DSG | MCUO_RELAY_DSG | MCUO_RELAY_MAIN)) {
						if(++su16_CBCInitDelayCnt >= 5) {
							su16_CBCInitDelayCnt = 0;
							InitCBC(NVIC_OPEN);
							su8_CBCStatus_Flag = OPEN;
						}
					}
					else {
						if(su16_CBCInitDelayCnt)su16_CBCInitDelayCnt = 0;	//突然过流之类的把其关掉了，重新计数
					}
					break;
					
				case OPEN:
					if(CLOSE == (MCUO_MOS_DSG | MCUO_RELAY_DSG | MCUO_RELAY_MAIN)) {
						InitCBC(NVIC_CLOSE);
						su8_CBCStatus_Flag = CLOSE;
					}
					break;
					
				default:
					break;
			}
			
			CBC_TimeCtrl_Relay(); 					//不放出去了，放这里好点
			break;
			
		default:
			su8_CBCStartUp_Flag = 0;
			break;
	}
}


void CBC_TimeCtrl_MOS(void) {
	static UINT16 su16_CBC_DSG_TCnt = 0;
	static UINT16 su16_CBC_Recover_TCnt = 0;

	if(0 == g_st_SysTimeFlag.bits.b1Sys10msFlag1) {
		return;
	}

	if(CBC_Element.u8CBC_DSG_ErrFlag) {
		if(CBC_Element.u8CBC_DSG_Cnt >= 3) {
			System_OnOFF_Func.bits.b1OnOFF_MOS_Relay = 0;
			ChargerLoad_Func.bits.b1OFFDriver_CBC = 1;
			CBC_Element.u8CBC_DSG_Cnt = 0;
			CBC_Element.u8CBC_DSG_ErrFlag = 0;

			System_ERROR_UserCallback(ERROR_DSG_SHORT);		//连续触发三次，判定为短路保护
		}
		else {
			if(++su16_CBC_DSG_TCnt >= 3) {
				su16_CBC_DSG_TCnt = 0;
				++CBC_Element.u8CBC_DSG_Cnt;
				CBC_Element.u8CBC_DSG_ErrFlag = 0;

				MCUO_MOS_DSG = OPEN;		//尝试打开
				MCUO_RELAY_DSG = OPEN;
				MCUO_RELAY_PRE = OPEN;
				MCUO_RELAY_MAIN = OPEN;
			}
		}
		
		if(su16_CBC_Recover_TCnt)su16_CBC_Recover_TCnt = 0;
	}
	else {
		if(CBC_Element.u8CBC_DSG_Cnt) {
			if(++su16_CBC_Recover_TCnt >= 100) {			//打开1s没问题，则清除计数
				su16_CBC_Recover_TCnt = 0;
				CBC_Element.u8CBC_DSG_Cnt = 0;
			}
		}
	}
}


void CBC_MOS(void) {
	static UINT8 su8_CBCStartUp_Flag = 0;

	if(SystemStatus.bits.b1StartUpBMS) {	//意味着管子已经可以开始判断是否打开还是关闭
		return;								//接触器打开是否会出现干扰的问题？还是说要完全打开才打开这个中断？后续观察
	}

	switch(su8_CBCStartUp_Flag) {
		case 0:
			InitCBC(NVIC_INIT);
			InitCBC(NVIC_OPEN);
			su8_CBCStartUp_Flag = 1;
			break;
			
		case 1:
			CBC_TimeCtrl_MOS(); 			//不放出去了，放这里好点
			break;
			
		default:
			su8_CBCStartUp_Flag = 0;
			break;
	}
}


void App_CBC(void) {
	#ifdef _MOS
	CBC_MOS();
	#elif _Relay
	CBC_Relay();
	#endif
}


void App_SysTime(void) {
	static UINT8 s_u8Cnt1ms = 0;

	static UINT8 s_u8Cnt10ms = 0;
	//static UINT8 s_u8Cnt20ms = 0;
	static UINT8 s_u8Cnt50ms = 0;
	static UINT8 s_u8Cnt100ms = 0;
	
	static UINT8 s_u8Cnt200ms1 = 0;
	static UINT8 s_u8Cnt200ms2 = 4;
	static UINT8 s_u8Cnt200ms3 = 8;
	static UINT8 s_u8Cnt200ms4 = 12;
	static UINT8 s_u8Cnt200ms5 = 16;

	static UINT8 s_u8Cnt1000ms1 = 0;
	static UINT8 s_u8Cnt1000ms2 = 33;
	static UINT8 s_u8Cnt1000ms3 = 66;

	g_st_SysTimeFlag.bits.b1Sys1msFlag = 0;
	if(s_u8Cnt1ms != g_u81msClockCnt) {				//1ms定时标志
		s_u8Cnt1ms = g_u81msClockCnt;
		g_st_SysTimeFlag.bits.b1Sys1msFlag = 1;
	}

	g_st_SysTimeFlag.bits.b1Sys10msFlag1 = 0;  
	g_st_SysTimeFlag.bits.b1Sys10msFlag2 = 0;
	g_st_SysTimeFlag.bits.b1Sys10msFlag3 = 0;
	g_st_SysTimeFlag.bits.b1Sys10msFlag4 = 0;
	g_st_SysTimeFlag.bits.b1Sys10msFlag5 = 0;
	if(s_u8Cnt10ms != g_u810msClockCnt) {		//10ms定时标志
		s_u8Cnt10ms = g_u810msClockCnt;
		switch(g_u810msClockCnt) {
			case 0:
				g_st_SysTimeFlag.bits.b1Sys10msFlag1 = 1;
				break;

			case 1:
				s_u8Cnt50ms++;
				g_st_SysTimeFlag.bits.b1Sys10msFlag2 = 1;
				break;

			case 2:
				s_u8Cnt100ms++;
				g_st_SysTimeFlag.bits.b1Sys10msFlag3 = 1;
				break;

			case 3:
				s_u8Cnt200ms1++;		//本想用一个变量搞一个循环然后置位，发现有BUG，不行
				s_u8Cnt200ms2++;		//会持续进来一个10ms，必须改变标志位让其不再进来
				s_u8Cnt200ms3++;
				s_u8Cnt200ms4++;
				s_u8Cnt200ms5++;
				g_st_SysTimeFlag.bits.b1Sys10msFlag4 = 1;
				break;

			case 4:
				s_u8Cnt1000ms1++;
				s_u8Cnt1000ms2++;
				s_u8Cnt1000ms3++;
				g_st_SysTimeFlag.bits.b1Sys10msFlag5 = 1;
				break;

			default:
				break;
		}
	}

	g_st_SysTimeFlag.bits.b1Sys50msFlag = 0;
	if(s_u8Cnt50ms >= 5) {
		s_u8Cnt50ms = 0;
		g_st_SysTimeFlag.bits.b1Sys50msFlag = 1;		//50ms定时标志
	}


	g_st_SysTimeFlag.bits.b1Sys100msFlag = 0;
	if(s_u8Cnt100ms >= 10) {
		s_u8Cnt100ms = 0;
		g_st_SysTimeFlag.bits.b1Sys100msFlag = 1;		//100ms定时标志
	}

	g_st_SysTimeFlag.bits.b1Sys200msFlag1 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag2 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag3 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag4 = 0;
	g_st_SysTimeFlag.bits.b1Sys200msFlag5 = 0;
	if(s_u8Cnt200ms1 >= 20) {
		s_u8Cnt200ms1 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag1 = 1;		//200ms定时标志
	}
	if(s_u8Cnt200ms2 >= 20) {
		s_u8Cnt200ms2 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag2 = 1;		//200ms定时标志
	}
	if(s_u8Cnt200ms3 >= 20) {
		s_u8Cnt200ms3 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag3 = 1;		//200ms定时标志
	}
	if(s_u8Cnt200ms4 >= 20) {
		s_u8Cnt200ms4 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag4 = 1;		//200ms定时标志
		MCUO_DEBUG_LED1 = !MCUO_DEBUG_LED1;
	}
	if(s_u8Cnt200ms5 >= 20) {
		s_u8Cnt200ms5 = 0;
		g_st_SysTimeFlag.bits.b1Sys200msFlag5 = 1;		//200ms定时标志
	}


	g_st_SysTimeFlag.bits.b1Sys1000msFlag1 = 0;
	g_st_SysTimeFlag.bits.b1Sys1000msFlag2 = 0;
	g_st_SysTimeFlag.bits.b1Sys1000msFlag3 = 0;
	if(s_u8Cnt1000ms1 >= 100) {
		s_u8Cnt1000ms1 = 0;
		g_st_SysTimeFlag.bits.b1Sys1000msFlag1 = 1;		//1000ms定时标志
	}
	if(s_u8Cnt1000ms2 >= 100) {
		s_u8Cnt1000ms2 = 0;
		g_st_SysTimeFlag.bits.b1Sys1000msFlag2 = 1;		//1000ms定时标志
	}
	if(s_u8Cnt1000ms3 >= 100) {
		s_u8Cnt1000ms3 = 0;
		g_st_SysTimeFlag.bits.b1Sys1000msFlag3 = 1;		//1000ms定时标志
	}
}


void TIM17_IRQHandler(void) {
	if (TIM_GetITStatus(TIM17, TIM_IT_Update) != RESET) {  	//检查TIM3更新中断发生与否
		TIM_ClearITPendingBit(TIM17, TIM_IT_Update);  		//清除TIMx更新中断标志
		if((++g_u81msCnt) >= 2) {                           //1ms
            g_u81msCnt = 0;
            g_u81msClockCnt++;
			gu8_200msCnt++;

            if(g_u81msClockCnt >= 2) {                      	//2ms
                g_u81msClockCnt = 0;
                g_u810msClockCnt++;
                if(g_u810msClockCnt >= 5) {                 	//10ms
                    g_u810msClockCnt = 0;
                }
            }

			if(gu8_200msCnt >= 200) {
				gu8_200msCnt = 0;
				gu8_200msAccClock_Flag = 1;
			}
        }
	}
}

