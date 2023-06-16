#include "main.h"

volatile union SLEEP_MODE Sleep_Mode; // 用于外部控制进入休眠标志位
enum SLEEP_STATUS Sleep_Status = SLEEP_HICCUP_SHIFT;

UINT8 gu8_SleepStatus = 0;
UINT8 RTC_ExtComCnt = 0;

// 通讯唤醒对深度休眠不起效果。不能再Base加入通讯唤醒。
void InitWakeUp_Base(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 使能PWR外设时钟，待机模式，RTC，看门狗
	// PA0_WKUP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // 选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 设置引脚模式为上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 设置中断线0，EXTI0和PA0挂钩
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// 配置PA0_WKUP外部上升沿中断
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // 上升沿中断
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// 中断嵌套设计
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn; // 使能按键WK_UP所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00; // 抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	   // 使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; // 选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 设置引脚模式为上拉输入模式
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// 设置中断线0，EXTI0和PA0挂钩
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
	// 配置PA0_WKUP外部上升沿中断
	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // 上升沿中断
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// 中断嵌套设计
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn; // 使能按键WK_UP所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;	// 抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// 使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);
}

void InitWakeUp_NormalMode(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	InitWakeUp_Base();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // 开启GPIOB的外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 使能PWR外设时钟，待机模式，RTC，看门狗

	// PB7_INT_WK_CMNT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // 选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 设置引脚模式为上拉输入模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
	// 配置PA1_WKUP外部上升沿中断
	EXTI_InitStruct.EXTI_Line = EXTI_Line7;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // 上升沿中断
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// 中断嵌套设计
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn; // 使能按键WK_UP所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;	// 抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// 使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

	// //PA12_AFE唤醒
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//选择要用的GPIO引脚
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	//设置引脚模式为上拉输入模式
	// GPIO_Init(GPIOA, &GPIO_InitStructure);

	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);
	// //配置PA1_WKUP外部上升沿中断
	// EXTI_InitStruct.EXTI_Line = EXTI_Line12;
	// EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	// EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; //上升沿中断
	// EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	// EXTI_Init(&EXTI_InitStruct);
	// //中断嵌套设计
	// NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;	//使能按键WK_UP所在的外部中断通道
	// NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;	//抢占优先级0
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//使能外部中断通道
	// NVIC_Init(&NVIC_InitStructure);
}

// 在RTC的基础上加上normal的唤醒模式
void InitWakeUp_RTCMode(void)
{
	// InitWakeUp_Base();
	InitWakeUp_NormalMode(); // 包含了Base的唤醒方式
	RTC_TimeConfig();
	RTC_AlarmConfig();
}

// 如果是standby模式的话，PA0的wkup不用配
// 通讯唤醒对深度休眠不能起效果。
void InitWakeUp_DeepMode(void)
{
	InitWakeUp_Base();
}

void delay(int n)
{
	while (n--)
		;
}

void IOstatus_Base(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // 开启GPIOA的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // 开启GPIOB的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // 开启GPIOC的外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE); // 开启GPIOF的外设时钟

	//	/* 让AFE1进入ship模式 */
	// InitAFE1_Sleep();
	ADC_DeInit(ADC1);

	GPIOA->PUPDR = 0;
	GPIOA->MODER = 0XFFFFFFFF;
	GPIOB->PUPDR = 0;
	GPIOB->MODER = 0XFFFFFFFF;
	GPIOC->PUPDR = 0;
	GPIOC->MODER = 0XFFFFFFFF;
	GPIOF->PUPDR = 0;
	GPIOF->MODER = 0XFFFFFFFF;

	/*
	//没用，还是要把CLTL的影响去掉。不然各种休眠断一下，开机断一下。
	//PB14_CTLC_保持输出高电平，不然会断，上面的操作ns的断应该问题不大。
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);
	*/

	// PB5_PWSV_LDO
	// LDO不用管了，默认供电，测试过就是正常运行和休眠，功耗都不高(4s的情况下)
	//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	//  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//  GPIO_ResetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);
	//  delay(1500000);

	// PB1_PWSV_STB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);

	// PB15_MCUO_PWSV_CTR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);

	delay(1000000);
	// AFE_Sleep();
}

// 第二级休眠的AFE只有Sleep模式
// 第二级还是不要仓运模式，有可能要通过Alarm信号唤醒
void IOstatus_NormalMode(void)
{
	InitAFE1_Sleep(0);
	AFE_Sleep();
	IOstatus_Base();
}

// 高压的时候，默认是进入IDLE模式，如果IDLE模式不允许进去，则还是进Sleep模式，但是MCU还是RTC模式
void IOstatus_RTCMode(void)
{
	InitAFE1_Sleep(1);
	AFE_IDLE();
	IOstatus_Base();
}

// 深度休眠为SHIP模式
void IOstatus_DeepMode(void)
{
	// AFE_SHIP();			//不需要准备，直接控引脚进入ship模式

	InitAFE1_Sleep(0);
	AFE_Sleep();
	IOstatus_Base();
}

void IORecover_RTCMode(void)
{
	MCU_RESET();
}

void IORecover_NormalMode(void)
{
	// TIM_Cmd(TIM3, ENABLE);	//用于App_SleepTest()函数
	MCU_RESET(); // 由于直接走下去导致各种因为现场破坏无法进入正常工作模式，完美的解决办法是复位再来过
}

void IORecover_DeepMode(void)
{
	MCU_RESET();
}

// wkup不用配
void Sys_StandbyMode(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // 使能PWR外设时钟，这句话是否需要？030不需要也能进入休眠
	// RCC_APB2PeriphResetCmd(0X01FC,DISABLE);				//复位所有IO口   //TODO
	PWR_WakeUpPinCmd(PWR_WakeUpPin_1, ENABLE); // 使能唤醒管脚功能，PWR_CSR。
											   // 该引脚会被强制配置为下拉输入，意味着不需要配置了？

	PWR_ClearFlag(PWR_FLAG_WU); // Clear WUF bit in Power Control/Status register (PWR_CSR)
								// 清PWR_CR相关便能清除PWR_CSR
	PWR_EnterSTANDBYMode(); // 进入待命（STANDBY）模式，PWR_CR    _PDDS
							// SCB->SCR设置为SLEEPDEEP = 1
}

// 030也是这样吗
void Sys_StopMode(void)
{
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

// 如果信号到了，没法唤醒，单片机假死状态。
// 就是以下这段话执行出问题了，外部晶振出问题
#if (defined _HSE_8M_PLL_48M) || (defined _HSE_12M_PLL_48M)
	RCC_HSEConfig(RCC_HSE_ON); // 起来后会被切换回HSI
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
		;				// 等待 HSE 准备就绪
	RCC_PLLCmd(ENABLE); // 使能 PLL
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		;									   // 等待 PLL 准备就绪
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // 选择PLL作为系统时钟源
	while (RCC_GetSYSCLKSource() != 0x08)
		; // 等待PLL被选择为系统时钟源
#endif
}

void SleepDeal_Continue(void)
{
	UINT8 u8FlashWriteOK_flag = 0;
	static UINT8 s_u8SleepModeSelect = NORMAL_MODE;

	if (Sleep_Mode.bits.b1TestSleep)
	{
		s_u8SleepModeSelect = NORMAL_MODE;
	}
	else if (Sleep_Mode.bits.b1OverCurSleep)
	{
		s_u8SleepModeSelect = DEEP_MODE;
	}
	else if (Sleep_Mode.bits.b1OverVdeltaSleep)
	{
		s_u8SleepModeSelect = DEEP_MODE;
	}
	else if (Sleep_Mode.bits.b1CBCSleep)
	{
		s_u8SleepModeSelect = DEEP_MODE;
	}
	else if (Sleep_Mode.bits.b1ForceToSleep_L1)
	{
		s_u8SleepModeSelect = HICCUP_MODE;
	}
	else if (Sleep_Mode.bits.b1ForceToSleep_L2)
	{
		s_u8SleepModeSelect = NORMAL_MODE;
	}
	else if (Sleep_Mode.bits.b1ForceToSleep_L3)
	{
		s_u8SleepModeSelect = DEEP_MODE;
	}
	else if (Sleep_Mode.bits.b1VcellOVP)
	{
		// s_u8SleepModeSelect = HICCUP_MODE;
		s_u8SleepModeSelect = DEEP_MODE;
	}
	else if (Sleep_Mode.bits.b1VcellUVP)
	{
		// s_u8SleepModeSelect = HICCUP_MODE;
		s_u8SleepModeSelect = DEEP_MODE;
	}
	else if (Sleep_Mode.bits.b1NormalSleep_L1)
	{
		s_u8SleepModeSelect = HICCUP_MODE;
	}
	else if (Sleep_Mode.bits.b1NormalSleep_L2)
	{
		s_u8SleepModeSelect = NORMAL_MODE;
	}
	else if (Sleep_Mode.bits.b1NormalSleep_L3)
	{
		s_u8SleepModeSelect = DEEP_MODE;
	}
	else
	{
		s_u8SleepModeSelect = NORMAL_MODE;
	}

	switch (s_u8SleepModeSelect)
	{
	case NORMAL_MODE:
		if (FLASH_COMPLETE == FlashWriteOneHalfWord(FLASH_ADDR_SLEEP_FLAG, FLASH_NORMAL_SLEEP_VALUE))
		{
			u8FlashWriteOK_flag = 1;
		}
		break;
	case HICCUP_MODE:
		if (FLASH_COMPLETE == FlashWriteOneHalfWord(FLASH_ADDR_SLEEP_FLAG, FLASH_HICCUP_SLEEP_VALUE))
		{
			u8FlashWriteOK_flag = 1;
		}
		break;
	case DEEP_MODE:
		if (FLASH_COMPLETE == FlashWriteOneHalfWord(FLASH_ADDR_SLEEP_FLAG, FLASH_DEEP_SLEEP_VALUE))
		{
			u8FlashWriteOK_flag = 1;
		}
		break;
	default:
		// 不调整引脚进入休眠，功耗会很大
		break;
	}

	if (u8FlashWriteOK_flag)
	{
		MCU_RESET();
	}
}

void SleepDeal_OverCurrent(void)
{
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;

	if (!Sleep_Mode.bits.b1OverCurSleep)
	{									   // 加强雍余设计
		Sleep_Status = SLEEP_HICCUP_SHIFT; // 其实这个可以不要，设计要求，除了这个函数可以把这个标志位去除外，别的地方不可以便可
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // 留下位置，后续第一次后进入需要延时则这里加
			s_u32SleepFirstCnt = 0;
			s_u8SleepStatus = HICCUP;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	case HICCUP:
		if (++s_u32SleepHiccupCnt > SleepInitOC)
		{
			s_u32SleepHiccupCnt = 0;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	default:
		s_u8SleepStatus = FIRST; // 下个回合再来
		break;
	}

	if (0)
	{										// 如果检测到没问题，则退出休眠
		Sleep_Mode.bits.b1OverCurSleep = 0; // 放到switch语句外面，FIRST和HICCUP两个都有效
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
}

void SleepDeal_VcellOVP(void)
{
}

void SleepDeal_VcellUVP(void)
{
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;

	if (!Sleep_Mode.bits.b1VcellUVP)
	{									   // 加强雍余设计
		Sleep_Status = SLEEP_HICCUP_SHIFT; // 其实这个可以不要，设计要求，除了这个函数可以把这个标志位去除外，别的地方不可以便可
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // 直接进去
			s_u32SleepFirstCnt = 0;
			s_u8SleepStatus = HICCUP;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	case HICCUP:
		if (++s_u32SleepHiccupCnt > 0)
		{
			s_u32SleepHiccupCnt = 0;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	default:
		s_u8SleepStatus = FIRST; // 下个回合再来
		break;
	}

	if (0)
	{ // 如果检测到没问题，则退出休眠
		// Sleep_Mode.bits.b1ForceToSleep = 0;
		// Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST; // 直接回到第一次，force只有一次，不是打嗝休眠模式
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
}

void SleepDeal_Vdelta(void)
{
#if 0
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;
	
	if(!Sleep_Mode.bits.b1OverVdeltaSleep) {	//加强雍余设计
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		return ;
	}
#endif
}

void SleepDeal_Forced(void)
{
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;

	if (!Sleep_Mode.bits.b1ForceToSleep_L1 && Sleep_Mode.bits.b1ForceToSleep_L2 && Sleep_Mode.bits.b1ForceToSleep_L3)
	{									   // 加强雍余设计
		Sleep_Status = SLEEP_HICCUP_SHIFT; // 其实这个可以不要，设计要求，除了这个函数可以把这个标志位去除外，别的地方不可以便可
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // 直接进去
			s_u32SleepFirstCnt = 0;
			s_u8SleepStatus = HICCUP;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	case HICCUP:
		if (++s_u32SleepHiccupCnt > 0)
		{
			s_u32SleepHiccupCnt = 0;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	default:
		s_u8SleepStatus = FIRST; // 下个回合再来
		break;
	}

	if (0)
	{ // 如果检测到没问题，则退出休眠
		// Sleep_Mode.bits.b1ForceToSleep = 0;
		// Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST; // 直接回到第一次，force只有一次，不是打嗝休眠模式
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
}

void SleepDeal_CBC(void)
{
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;

	if (!Sleep_Mode.bits.b1CBCSleep)
	{									   // 加强雍余设计
		Sleep_Status = SLEEP_HICCUP_SHIFT; // 其实这个可以不要，设计要求，除了这个函数可以把这个标志位去除外，别的地方不可以便可
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // 留下位置，后续第一次后进入需要延时则这里加
			s_u32SleepFirstCnt = 0;
			s_u8SleepStatus = HICCUP;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	case HICCUP:
		if (++s_u32SleepHiccupCnt > SleepInitCBC)
		{
			s_u32SleepHiccupCnt = 0;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	default:
		s_u8SleepStatus = FIRST; // 下个回合再来
		break;
	}

	if (0)
	{									// 如果检测到没问题，则退出休眠
		Sleep_Mode.bits.b1CBCSleep = 0; // 放到switch语句外面，FIRST和HICCUP两个都有效
		// System_OnOFF_Func.bits.b1OnOFF_MOS_Relay = 1; 		//在这里复原是否更好？
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
}

void SleepDeal_Normal_L1(void)
{
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;
	static UINT8 su8_SleepExtComCnt = 0;

	if ((Sleep_Mode.all & 0xFFF1) != 0)
	{ // 核心
		Sleep_Mode.bits.b1NormalSleep_L1 = 0;
		Sleep_Mode.bits.b1NormalSleep_L2 = 0;
		Sleep_Mode.bits.b1NormalSleep_L3 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
		return;
	}

	if (su8_SleepExtComCnt != RTC_ExtComCnt)
	{
		su8_SleepExtComCnt = RTC_ExtComCnt;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (OtherElement.u16Sleep_TimeRTC == 0)
		{
			// 为0时默认RTC不进入休眠
		}
		else
		{
			if (++s_u32SleepFirstCnt > (UINT32)OtherElement.u16Sleep_TimeRTC * 60)
			{
				// if(++s_u32SleepFirstCnt >= 5) {			//这个，第一次个后面都是一样
				s_u32SleepFirstCnt = 0;
				s_u8SleepStatus = HICCUP;
				Sleep_Status = SLEEP_HICCUP_CONTINUE;
			}
		}
		break;

	case HICCUP:
		if (++s_u32SleepHiccupCnt > (UINT32)OtherElement.u16Sleep_TimeRTC * 60)
		{
			s_u32SleepHiccupCnt = 0;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	default:
		s_u8SleepStatus = FIRST; // 下个回合再来
		break;
	}

	if (g_stCellInfoReport.u16Ichg > OtherElement.u16Sleep_VirCur_Chg || g_stCellInfoReport.u16IDischg > OtherElement.u16Sleep_VirCur_Dsg)
	{
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}

	if (g_stCellInfoReport.u16VCellMin <= OtherElement.u16Sleep_VNormal)
	{
		Sleep_Mode.bits.b1NormalSleep_L1 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
	// s_u32SleepFirstCnt = 0;		//还没调好L1不进入休眠。
}

void SleepDeal_Normal_L2(void)
{
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;
	static UINT8 su8_SleepExtComCnt = 0;

	if ((Sleep_Mode.all & 0xFFF1) != 0)
	{
		Sleep_Mode.bits.b1NormalSleep_L1 = 0;
		Sleep_Mode.bits.b1NormalSleep_L2 = 0;
		Sleep_Mode.bits.b1NormalSleep_L3 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
		return;
	}

	if (su8_SleepExtComCnt != RTC_ExtComCnt)
	{
		su8_SleepExtComCnt = RTC_ExtComCnt;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > (UINT32)OtherElement.u16Sleep_TimeNormal * 60)
		{
			// if(++s_u32SleepFirstCnt >= 3) {			//这个，第一次个后面都是一样
			s_u32SleepFirstCnt = 0;
			s_u8SleepStatus = HICCUP;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	case HICCUP:
		if (++s_u32SleepHiccupCnt > (UINT32)OtherElement.u16Sleep_TimeNormal * 60)
		{
			// if(++s_u32SleepHiccupCnt >= 1) {
			s_u32SleepHiccupCnt = 0;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	default:
		s_u8SleepStatus = FIRST; // 下个回合再来
		break;
	}

	if (g_stCellInfoReport.u16Ichg > OtherElement.u16Sleep_VirCur_Chg || g_stCellInfoReport.u16IDischg > OtherElement.u16Sleep_VirCur_Dsg)
	{
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}

	if (g_stCellInfoReport.u16VCellMin < OtherElement.u16Sleep_Vlow || g_stCellInfoReport.u16VCellMin > OtherElement.u16Sleep_VNormal)
	{ // 触发条件才跳转，别的时间不跳转
		Sleep_Mode.bits.b1NormalSleep_L2 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
}

void SleepDeal_Normal_L3(void)
{
	static UINT8 s_u8SleepStatus = FIRST;
	static UINT32 s_u32SleepFirstCnt = 0;
	static UINT32 s_u32SleepHiccupCnt = 0;
	// static UINT8 su8_SleepExtComCnt = 0;

	if ((Sleep_Mode.all & 0xFFF1) != 0)
	{
		Sleep_Mode.bits.b1NormalSleep_L1 = 0;
		Sleep_Mode.bits.b1NormalSleep_L2 = 0;
		Sleep_Mode.bits.b1NormalSleep_L3 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
		return;
	}

#if 0
	if(su8_SleepExtComCnt != RTC_ExtComCnt) {
		su8_SleepExtComCnt = RTC_ExtComCnt;
		if(s_u32SleepFirstCnt)s_u32SleepFirstCnt = 0;
		if(s_u32SleepHiccupCnt)s_u32SleepHiccupCnt = 0;
	}
#endif

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > (UINT32)OtherElement.u16Sleep_TimeVlow * 60)
		{
			// if(++s_u32SleepFirstCnt >= 1) {			//这个，第一次个后面都是一样
			s_u32SleepFirstCnt = 0;
			s_u8SleepStatus = HICCUP;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	case HICCUP:
		if (++s_u32SleepHiccupCnt > (UINT32)OtherElement.u16Sleep_TimeVlow * 60)
		{
			// if(++s_u32SleepHiccupCnt >= 1) {
			s_u32SleepHiccupCnt = 0;
			Sleep_Status = SLEEP_HICCUP_CONTINUE;
		}
		break;

	default:
		s_u8SleepStatus = FIRST; // 下个回合再来
		break;
	}

	if (g_stCellInfoReport.u16Ichg > OtherElement.u16Sleep_VirCur_Chg || g_stCellInfoReport.u16IDischg > OtherElement.u16Sleep_VirCur_Dsg)
	{
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}

	if (g_stCellInfoReport.u16VCellMin >= OtherElement.u16Sleep_Vlow)
	{ // 触发条件才跳转，别的时间不跳转
		Sleep_Mode.bits.b1NormalSleep_L3 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
}

// 这个地方，IO控制策略要改一下，起来延时1s再打开管子会不会更好？不过现象貌似直接打开没问题
// 这个作为主循环，如果开头判断出现了别的错误，则跳出主循环，去执行别的
// 关于这里和IO控制主函数的逻辑问题，A，最开头关于Sleep的return问题。B，休眠起来IO是否立刻打开的问题
void SleepDeal_Normal_Select(void)
{
	if ((Sleep_Mode.all & 0xFFF1) != 0)
	{ // 核心
		Sleep_Mode.bits.b1NormalSleep_L1 = 0;
		Sleep_Mode.bits.b1NormalSleep_L2 = 0;
		Sleep_Mode.bits.b1NormalSleep_L3 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		return;
	}

	if (g_stCellInfoReport.u16Ichg <= OtherElement.u16Sleep_VirCur_Chg && g_stCellInfoReport.u16IDischg <= OtherElement.u16Sleep_VirCur_Chg)
	{
		if (g_stCellInfoReport.u16VCellMin < OtherElement.u16Sleep_Vlow)
		{
			Sleep_Mode.bits.b1NormalSleep_L3 = 1;
			Sleep_Status = SLEEP_HICCUP_NORMAL_L3;
		}
		else if (g_stCellInfoReport.u16VCellMin > OtherElement.u16Sleep_VNormal)
		{
			Sleep_Mode.bits.b1NormalSleep_L1 = 1;
			Sleep_Status = SLEEP_HICCUP_NORMAL_L1;
		}
		else
		{ // 等号均纳入L2
			Sleep_Mode.bits.b1NormalSleep_L2 = 1;
			Sleep_Status = SLEEP_HICCUP_NORMAL_L2;
		}
	}
	else
	{
		// 有电流则继续在这个函数循环
	}
}

// 架构决定要改一改，不然后期人员太难维护了
void SleepDeal_Shift(void)
{
	if (Sleep_Mode.bits.b1TestSleep != 0)
	{
		Sleep_Status = SLEEP_HICCUP_TEST;
	}
	else if (Sleep_Mode.bits.b1OverCurSleep != 0)
	{
		// Sleep_Status = SLEEP_HICCUP_CONTINUE;			//架构已改，先跳到相关函数，再进入休眠
		Sleep_Status = SLEEP_HICCUP_OVERCUR;
	}
	else if (Sleep_Mode.bits.b1OverVdeltaSleep != 0)
	{
		Sleep_Status = SLEEP_HICCUP_OVDELTA;
	}
	else if (Sleep_Mode.bits.b1CBCSleep != 0)
	{
		Sleep_Status = SLEEP_HICCUP_CBC;
	}
	else if (Sleep_Mode.bits.b1ForceToSleep_L1 != 0)
	{
		Sleep_Status = SLEEP_HICCUP_FORCED;
	}
	else if (Sleep_Mode.bits.b1ForceToSleep_L2 != 0)
	{
		Sleep_Status = SLEEP_HICCUP_FORCED;
	}
	else if (Sleep_Mode.bits.b1ForceToSleep_L3 != 0)
	{
		Sleep_Status = SLEEP_HICCUP_FORCED;
	}
	else if (Sleep_Mode.bits.b1VcellOVP != 0)
	{
		Sleep_Status = SLEEP_HICCUP_VCELLOVP;
	}
	else if (Sleep_Mode.bits.b1VcellUVP != 0)
	{
		Sleep_Status = SLEEP_HICCUP_VCELLUVP;
	}
	else
	{ // 没有以上各种保护直接进入主循环
		Sleep_Status = SLEEP_HICCUP_NORMAL_SELECT;
	}
}

void SleepDeal_Test(void)
{
	static UINT16 s_u16HaltTestCnt = 0;
	if (!Sleep_Mode.bits.b1TestSleep)
	{ // 加强雍余设计
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		return;
	}

	if (++s_u16HaltTestCnt >= 2)
	{ // 10s——Test
		s_u16HaltTestCnt = 0;
		Sleep_Status = SLEEP_HICCUP_CONTINUE;
	}
}

// 如果起来了，有保护，作别的操作类型还没写
// 现在通过参数设置，高于1005为RTC休眠
void IsSleepStartUp(void)
{
	switch (FlashReadOneHalfWord(FLASH_ADDR_SLEEP_FLAG))
	{
	case FLASH_HICCUP_SLEEP_VALUE:
		if (FLASH_COMPLETE == FlashWriteOneHalfWord(FLASH_ADDR_SLEEP_FLAG, FLASH_SLEEP_RESET_VALUE))
		{
			InitIO();
			InitDelay();
			InitSystemWakeUp();
			InitE2PROM(); // 内部EEPROM，不需要初始化
			Init_RTC();

			IOstatus_RTCMode();
			InitWakeUp_RTCMode();
			Sys_StopMode();
			// Sys_StandbyMode();
			IORecover_RTCMode();
		}
		break;
	case FLASH_NORMAL_SLEEP_VALUE:
		if (FLASH_COMPLETE == FlashWriteOneHalfWord(FLASH_ADDR_SLEEP_FLAG, FLASH_SLEEP_RESET_VALUE))
		{
			InitIO();
			InitDelay();
			InitSystemWakeUp();

			IOstatus_NormalMode();
			InitWakeUp_NormalMode();
			Sys_StopMode();
			IORecover_NormalMode();
		}
		break;
	case FLASH_DEEP_SLEEP_VALUE:
		if (FLASH_COMPLETE == FlashWriteOneHalfWord(FLASH_ADDR_SLEEP_FLAG, FLASH_SLEEP_RESET_VALUE))
		{
			InitIO();
			InitDelay();
			InitSystemWakeUp();

			IOstatus_DeepMode();
			InitWakeUp_DeepMode();
			// Sys_StandbyMode();		//不能掌控外部IO，弃用
			Sys_StopMode();
			IORecover_DeepMode();
		}
		break;
	case FLASH_SLEEP_RESET_VALUE:
		// 不作处理
		break;
	default:
		break;
	}
}

// 各种状况进入休眠(多个循环，例如正常休眠循环，CBC休眠循环，压差过大保护循环)
// 通过SleepDeal_NormalQuit()-->主控跳转函数，跳到别的循环
// Sleep_Mode标志位是进入哪种休眠循环，外部决定，出现别的休眠状况，立刻跳过去先进入休眠，然后自动唤醒，再回到相关函数循环。
// 所以，流程是：
// Sleep_Mode标志-->SleepDeal_Normal(正常循环)-->SleepDeal_NormalQuit(跳转)-->SleepDeal_Continue(休眠)-->唤醒进入相关循环函数
// 以上架构因太过复杂太过难以被后续人员维护，不太契合实际流程，已被修改为如下。
// 唤醒进入相关循环函数，含有第一次FIRST和后续HICCUP模式进入两种情况，所以第一次能立刻进入，第二次开始打嗝进入
// Sleep_Mode标志-->SleepDeal_Normal(正常循环)-->SleepDeal_NormalQuit(跳转)-->唤醒进入相关循环函数-->SleepDeal_Continue(休眠)
void App_SleepDeal(void)
{
	if (!System_OnOFF_Func.bits.b1OnOFF_Sleep)
	{			// 有个疑问，是不是立刻关了，不需要复原呢，均衡是需要关掉复原。
		return; // Sleep的话，如果直接不进去，后续打开会接着上次的步伐
	}			// 暂且先这么做，后续如果要全盘复原，计时清零再说，目前是接着上次的步伐

	if (SystemStatus.bits.b1StartUpBMS)
	{ // 开机完毕再进入
		return;
	}
	else
	{
		SystemStatus.bits.b1Status_ToSleep = 1;
	}

	if (Sleep_Mode.bits.b1_ToSleepFlag)
	{
		LogRecord_Flag.bits.Log_Sleep = 1;
		return;
	}

	if (0 == g_st_SysTimeFlag.bits.b1Sys1000msFlag1 && !Sleep_Mode.bits.b1ForceToSleep_L1 && !Sleep_Mode.bits.b1ForceToSleep_L2 && !Sleep_Mode.bits.b1ForceToSleep_L3)
	{
		return; // 如果是强制进入休眠的则必须快点进入休眠，不能拖
	}

	switch (Sleep_Status)
	{
	case SLEEP_HICCUP_SHIFT: // 先跳到这里，再跳到SleepDeal_Continue()，然后进入别的循环
		SleepDeal_Shift();	 // 主控跳转函数，开机执行一遍没事进入核心循环函数
		break;
	case SLEEP_HICCUP_NORMAL_SELECT:
		SleepDeal_Normal_Select();
		break;
	case SLEEP_HICCUP_TEST:
		SleepDeal_Test();
		break;
	case SLEEP_HICCUP_OVERCUR:
		SleepDeal_OverCurrent();
		break;
	case SLEEP_HICCUP_OVDELTA:
		SleepDeal_Vdelta(); // 目前压差过大直接进入休眠不起来，亮个灯
		break;
	case SLEEP_HICCUP_CBC:
		SleepDeal_CBC();
		break;
	case SLEEP_HICCUP_FORCED:
		SleepDeal_Forced(); // 还没写
		break;
	case SLEEP_HICCUP_NORMAL_L1:
		SleepDeal_Normal_L1();
		break;
	case SLEEP_HICCUP_NORMAL_L2:
		SleepDeal_Normal_L2();
		break;
	case SLEEP_HICCUP_NORMAL_L3:
		SleepDeal_Normal_L3();
		break;

	case SLEEP_HICCUP_VCELLOVP:
		SleepDeal_VcellOVP();
		break;
	case SLEEP_HICCUP_VCELLUVP:
		SleepDeal_VcellUVP();
		break;

	case SLEEP_HICCUP_CONTINUE:
		SleepDeal_Continue();
		break;
	default:
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		break;
	}

	if (SLEEP_HICCUP_CONTINUE == Sleep_Status)
	{
		Sleep_Mode.bits.b1_ToSleepFlag = 1;
	}
	else
	{
		Sleep_Mode.bits.b1_ToSleepFlag = 0;
	}
}

void IOstatus_TestMode(void)
{
	IOstatus_NormalMode();
}

void InitWakeUp_TestMode(void)
{
	InitWakeUp_NormalMode();
}

void IORecover_TestMode(void)
{
	MCU_RESET();
}

void App_NormalSleepTest(void)
{
	static UINT16 s_u16HaltTestCnt = 0;

	if (0 == g_st_SysTimeFlag.bits.b1Sys1000msFlag1)
	{ // 休眠起来等待系统初始化完成
		return;
	}

	if (++s_u16HaltTestCnt >= 5)
	{ // 10s——Test
		s_u16HaltTestCnt = 0;
		IOstatus_TestMode();
		InitWakeUp_TestMode();
		TIM_Cmd(TIM17, DISABLE); //
		Sys_StopMode();
		// Sys_StandbyMode();
		IORecover_TestMode();
	}
}

void Sys_SleepOnExitMode(void)
{
	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE); // 库函数版本，设置SLEEP ON EXIT位为1
	// SCB->SCR|=1<<1;//寄存器版本，设置SLEEP ON EXIT位为1
	__ASM volatile("wfi");
}

void App_RTCSleepTest(void)
{
	static UINT16 s_u16HaltTestCnt = 0;

	if (0 == g_st_SysTimeFlag.bits.b1Sys1000msFlag1)
	{ // 休眠起来等待系统初始化完成
		return;
	}

	if (++s_u16HaltTestCnt >= 5)
	{ // 10s——Test
		s_u16HaltTestCnt = 0;
		// IOstatus_RTCMode();
		// InitWakeUp_RTCMode();
		// Sys_StopMode();
		// IORecover_RTCMode();
		Sleep_Mode.bits.b1ForceToSleep_L2 = 1;

		/*
		InitIO();
		InitSystemWakeUp();
		Init_RTC();
		InitE2PROM();			//这个用于RTC的唤醒数值的读取，EEPROM
		IOstatus_RTCMode();
		InitWakeUp_RTCMode();
		Sys_StopMode();
		//Sys_StandbyMode();
		IORecover_RTCMode();
		*/
	}
}
