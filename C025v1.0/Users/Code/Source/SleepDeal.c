#include "main.h"

volatile union SLEEP_MODE Sleep_Mode; // �����ⲿ���ƽ������߱�־λ
enum SLEEP_STATUS Sleep_Status = SLEEP_HICCUP_SHIFT;

UINT8 gu8_SleepStatus = 0;
UINT8 RTC_ExtComCnt = 0;

// ͨѶ���Ѷ�������߲���Ч����������Base����ͨѶ���ѡ�
void InitWakeUp_Base(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // ʹ��PWR����ʱ�ӣ�����ģʽ��RTC�����Ź�
	// PA0_WKUP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��������ģʽΪ��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// �����ж���0��EXTI0��PA0�ҹ�
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// ����PA0_WKUP�ⲿ�������ж�
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // �������ж�
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// �ж�Ƕ�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn; // ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00; // ��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	   // ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; // ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��������ģʽΪ��������ģʽ
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// �����ж���0��EXTI0��PA0�ҹ�
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
	// ����PA0_WKUP�ⲿ�������ж�
	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // �������ж�
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// �ж�Ƕ�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn; // ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;	// ��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);
}

void InitWakeUp_NormalMode(void)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	InitWakeUp_Base();

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // ����GPIOB������ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // ʹ��PWR����ʱ�ӣ�����ģʽ��RTC�����Ź�

	// PB7_INT_WK_CMNT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��������ģʽΪ��������ģʽ
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
	// ����PA1_WKUP�ⲿ�������ж�
	EXTI_InitStruct.EXTI_Line = EXTI_Line7;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // �������ж�
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	// �ж�Ƕ�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn; // ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;	// ��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);

	// //PA12_AFE����
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		//ѡ��Ҫ�õ�GPIO����
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	//��������ģʽΪ��������ģʽ
	// GPIO_Init(GPIOA, &GPIO_InitStructure);

	// SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);
	// //����PA1_WKUP�ⲿ�������ж�
	// EXTI_InitStruct.EXTI_Line = EXTI_Line12;
	// EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	// EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; //�������ж�
	// EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	// EXTI_Init(&EXTI_InitStruct);
	// //�ж�Ƕ�����
	// NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;	//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
	// NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;	//��ռ���ȼ�0
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//ʹ���ⲿ�ж�ͨ��
	// NVIC_Init(&NVIC_InitStructure);
}

// ��RTC�Ļ����ϼ���normal�Ļ���ģʽ
void InitWakeUp_RTCMode(void)
{
	// InitWakeUp_Base();
	InitWakeUp_NormalMode(); // ������Base�Ļ��ѷ�ʽ
	RTC_TimeConfig();
	RTC_AlarmConfig();
}

// �����standbyģʽ�Ļ���PA0��wkup������
// ͨѶ���Ѷ�������߲�����Ч����
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

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // ����GPIOA������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // ����GPIOB������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // ����GPIOC������ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE); // ����GPIOF������ʱ��

	//	/* ��AFE1����shipģʽ */
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
	//û�ã�����Ҫ��CLTL��Ӱ��ȥ������Ȼ�������߶�һ�£�������һ�¡�
	//PB14_CTLC_��������ߵ�ƽ����Ȼ��ϣ�����Ĳ���ns�Ķ�Ӧ�����ⲻ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);
	*/

	// PB5_PWSV_LDO
	// LDO���ù��ˣ�Ĭ�Ϲ��磬���Թ������������к����ߣ����Ķ�����(4s�������)
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

// �ڶ������ߵ�AFEֻ��Sleepģʽ
// �ڶ������ǲ�Ҫ����ģʽ���п���Ҫͨ��Alarm�źŻ���
void IOstatus_NormalMode(void)
{
	InitAFE1_Sleep(0);
	AFE_Sleep();
	IOstatus_Base();
}

// ��ѹ��ʱ��Ĭ���ǽ���IDLEģʽ�����IDLEģʽ�������ȥ�����ǽ�Sleepģʽ������MCU����RTCģʽ
void IOstatus_RTCMode(void)
{
	InitAFE1_Sleep(1);
	AFE_IDLE();
	IOstatus_Base();
}

// �������ΪSHIPģʽ
void IOstatus_DeepMode(void)
{
	// AFE_SHIP();			//����Ҫ׼����ֱ�ӿ����Ž���shipģʽ

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
	// TIM_Cmd(TIM3, ENABLE);	//����App_SleepTest()����
	MCU_RESET(); // ����ֱ������ȥ���¸�����Ϊ�ֳ��ƻ��޷�������������ģʽ�������Ľ���취�Ǹ�λ������
}

void IORecover_DeepMode(void)
{
	MCU_RESET();
}

// wkup������
void Sys_StandbyMode(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // ʹ��PWR����ʱ�ӣ���仰�Ƿ���Ҫ��030����ҪҲ�ܽ�������
	// RCC_APB2PeriphResetCmd(0X01FC,DISABLE);				//��λ����IO��   //TODO
	PWR_WakeUpPinCmd(PWR_WakeUpPin_1, ENABLE); // ʹ�ܻ��ѹܽŹ��ܣ�PWR_CSR��
											   // �����Żᱻǿ������Ϊ�������룬��ζ�Ų���Ҫ�����ˣ�

	PWR_ClearFlag(PWR_FLAG_WU); // Clear WUF bit in Power Control/Status register (PWR_CSR)
								// ��PWR_CR��ر������PWR_CSR
	PWR_EnterSTANDBYMode(); // ���������STANDBY��ģʽ��PWR_CR    _PDDS
							// SCB->SCR����ΪSLEEPDEEP = 1
}

// 030Ҳ��������
void Sys_StopMode(void)
{
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

// ����źŵ��ˣ�û�����ѣ���Ƭ������״̬��
// ����������λ�ִ�г������ˣ��ⲿ���������
#if (defined _HSE_8M_PLL_48M) || (defined _HSE_12M_PLL_48M)
	RCC_HSEConfig(RCC_HSE_ON); // ������ᱻ�л���HSI
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
		;				// �ȴ� HSE ׼������
	RCC_PLLCmd(ENABLE); // ʹ�� PLL
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		;									   // �ȴ� PLL ׼������
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // ѡ��PLL��Ϊϵͳʱ��Դ
	while (RCC_GetSYSCLKSource() != 0x08)
		; // �ȴ�PLL��ѡ��Ϊϵͳʱ��Դ
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
		// ���������Ž������ߣ����Ļ�ܴ�
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
	{									   // ��ǿӺ�����
		Sleep_Status = SLEEP_HICCUP_SHIFT; // ��ʵ������Բ�Ҫ�����Ҫ�󣬳�������������԰������־λȥ���⣬��ĵط������Ա��
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // ����λ�ã�������һ�κ������Ҫ��ʱ�������
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
		s_u8SleepStatus = FIRST; // �¸��غ�����
		break;
	}

	if (0)
	{										// �����⵽û���⣬���˳�����
		Sleep_Mode.bits.b1OverCurSleep = 0; // �ŵ�switch������棬FIRST��HICCUP��������Ч
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
	{									   // ��ǿӺ�����
		Sleep_Status = SLEEP_HICCUP_SHIFT; // ��ʵ������Բ�Ҫ�����Ҫ�󣬳�������������԰������־λȥ���⣬��ĵط������Ա��
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // ֱ�ӽ�ȥ
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
		s_u8SleepStatus = FIRST; // �¸��غ�����
		break;
	}

	if (0)
	{ // �����⵽û���⣬���˳�����
		// Sleep_Mode.bits.b1ForceToSleep = 0;
		// Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST; // ֱ�ӻص���һ�Σ�forceֻ��һ�Σ����Ǵ�������ģʽ
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
	
	if(!Sleep_Mode.bits.b1OverVdeltaSleep) {	//��ǿӺ�����
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
	{									   // ��ǿӺ�����
		Sleep_Status = SLEEP_HICCUP_SHIFT; // ��ʵ������Բ�Ҫ�����Ҫ�󣬳�������������԰������־λȥ���⣬��ĵط������Ա��
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // ֱ�ӽ�ȥ
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
		s_u8SleepStatus = FIRST; // �¸��غ�����
		break;
	}

	if (0)
	{ // �����⵽û���⣬���˳�����
		// Sleep_Mode.bits.b1ForceToSleep = 0;
		// Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST; // ֱ�ӻص���һ�Σ�forceֻ��һ�Σ����Ǵ�������ģʽ
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
	{									   // ��ǿӺ�����
		Sleep_Status = SLEEP_HICCUP_SHIFT; // ��ʵ������Բ�Ҫ�����Ҫ�󣬳�������������԰������־λȥ���⣬��ĵط������Ա��
		return;
	}

	switch (s_u8SleepStatus)
	{
	case FIRST:
		if (++s_u32SleepFirstCnt > 0)
		{ // ����λ�ã�������һ�κ������Ҫ��ʱ�������
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
		s_u8SleepStatus = FIRST; // �¸��غ�����
		break;
	}

	if (0)
	{									// �����⵽û���⣬���˳�����
		Sleep_Mode.bits.b1CBCSleep = 0; // �ŵ�switch������棬FIRST��HICCUP��������Ч
		// System_OnOFF_Func.bits.b1OnOFF_MOS_Relay = 1; 		//�����︴ԭ�Ƿ���ã�
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
	{ // ����
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
			// Ϊ0ʱĬ��RTC����������
		}
		else
		{
			if (++s_u32SleepFirstCnt > (UINT32)OtherElement.u16Sleep_TimeRTC * 60)
			{
				// if(++s_u32SleepFirstCnt >= 5) {			//�������һ�θ����涼��һ��
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
		s_u8SleepStatus = FIRST; // �¸��غ�����
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
	// s_u32SleepFirstCnt = 0;		//��û����L1���������ߡ�
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
			// if(++s_u32SleepFirstCnt >= 3) {			//�������һ�θ����涼��һ��
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
		s_u8SleepStatus = FIRST; // �¸��غ�����
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
	{ // ������������ת�����ʱ�䲻��ת
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
			// if(++s_u32SleepFirstCnt >= 1) {			//�������һ�θ����涼��һ��
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
		s_u8SleepStatus = FIRST; // �¸��غ�����
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
	{ // ������������ת�����ʱ�䲻��ת
		Sleep_Mode.bits.b1NormalSleep_L3 = 0;
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		s_u8SleepStatus = FIRST;
		if (s_u32SleepFirstCnt)
			s_u32SleepFirstCnt = 0;
		if (s_u32SleepHiccupCnt)
			s_u32SleepHiccupCnt = 0;
	}
}

// ����ط���IO���Ʋ���Ҫ��һ�£�������ʱ1s�ٴ򿪹��ӻ᲻����ã���������ò��ֱ�Ӵ�û����
// �����Ϊ��ѭ���������ͷ�жϳ����˱�Ĵ�����������ѭ����ȥִ�б��
// ���������IO�������������߼����⣬A���ͷ����Sleep��return���⡣B����������IO�Ƿ����̴򿪵�����
void SleepDeal_Normal_Select(void)
{
	if ((Sleep_Mode.all & 0xFFF1) != 0)
	{ // ����
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
		{ // �Ⱥž�����L2
			Sleep_Mode.bits.b1NormalSleep_L2 = 1;
			Sleep_Status = SLEEP_HICCUP_NORMAL_L2;
		}
	}
	else
	{
		// �е�����������������ѭ��
	}
}

// �ܹ�����Ҫ��һ�ģ���Ȼ������Ա̫��ά����
void SleepDeal_Shift(void)
{
	if (Sleep_Mode.bits.b1TestSleep != 0)
	{
		Sleep_Status = SLEEP_HICCUP_TEST;
	}
	else if (Sleep_Mode.bits.b1OverCurSleep != 0)
	{
		// Sleep_Status = SLEEP_HICCUP_CONTINUE;			//�ܹ��Ѹģ���������غ������ٽ�������
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
	{ // û�����ϸ��ֱ���ֱ�ӽ�����ѭ��
		Sleep_Status = SLEEP_HICCUP_NORMAL_SELECT;
	}
}

void SleepDeal_Test(void)
{
	static UINT16 s_u16HaltTestCnt = 0;
	if (!Sleep_Mode.bits.b1TestSleep)
	{ // ��ǿӺ�����
		Sleep_Status = SLEEP_HICCUP_SHIFT;
		return;
	}

	if (++s_u16HaltTestCnt >= 2)
	{ // 10s����Test
		s_u16HaltTestCnt = 0;
		Sleep_Status = SLEEP_HICCUP_CONTINUE;
	}
}

// ��������ˣ��б���������Ĳ������ͻ�ûд
// ����ͨ���������ã�����1005ΪRTC����
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
			InitE2PROM(); // �ڲ�EEPROM������Ҫ��ʼ��
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
			// Sys_StandbyMode();		//�����ƿ��ⲿIO������
			Sys_StopMode();
			IORecover_DeepMode();
		}
		break;
	case FLASH_SLEEP_RESET_VALUE:
		// ��������
		break;
	default:
		break;
	}
}

// ����״����������(���ѭ����������������ѭ����CBC����ѭ����ѹ����󱣻�ѭ��)
// ͨ��SleepDeal_NormalQuit()-->������ת�������������ѭ��
// Sleep_Mode��־λ�ǽ�����������ѭ�����ⲿ���������ֱ������״������������ȥ�Ƚ������ߣ�Ȼ���Զ����ѣ��ٻص���غ���ѭ����
// ���ԣ������ǣ�
// Sleep_Mode��־-->SleepDeal_Normal(����ѭ��)-->SleepDeal_NormalQuit(��ת)-->SleepDeal_Continue(����)-->���ѽ������ѭ������
// ���ϼܹ���̫������̫�����Ա�������Աά������̫����ʵ�����̣��ѱ��޸�Ϊ���¡�
// ���ѽ������ѭ�����������е�һ��FIRST�ͺ���HICCUPģʽ����������������Ե�һ�������̽��룬�ڶ��ο�ʼ���ý���
// Sleep_Mode��־-->SleepDeal_Normal(����ѭ��)-->SleepDeal_NormalQuit(��ת)-->���ѽ������ѭ������-->SleepDeal_Continue(����)
void App_SleepDeal(void)
{
	if (!System_OnOFF_Func.bits.b1OnOFF_Sleep)
	{			// �и����ʣ��ǲ������̹��ˣ�����Ҫ��ԭ�أ���������Ҫ�ص���ԭ��
		return; // Sleep�Ļ������ֱ�Ӳ���ȥ�������򿪻�����ϴεĲ���
	}			// ��������ô�����������Ҫȫ�̸�ԭ����ʱ������˵��Ŀǰ�ǽ����ϴεĲ���

	if (SystemStatus.bits.b1StartUpBMS)
	{ // ��������ٽ���
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
		return; // �����ǿ�ƽ������ߵ��������������ߣ�������
	}

	switch (Sleep_Status)
	{
	case SLEEP_HICCUP_SHIFT: // ���������������SleepDeal_Continue()��Ȼ�������ѭ��
		SleepDeal_Shift();	 // ������ת����������ִ��һ��û�½������ѭ������
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
		SleepDeal_Vdelta(); // Ŀǰѹ�����ֱ�ӽ������߲�������������
		break;
	case SLEEP_HICCUP_CBC:
		SleepDeal_CBC();
		break;
	case SLEEP_HICCUP_FORCED:
		SleepDeal_Forced(); // ��ûд
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
	{ // ���������ȴ�ϵͳ��ʼ�����
		return;
	}

	if (++s_u16HaltTestCnt >= 5)
	{ // 10s����Test
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
	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE); // �⺯���汾������SLEEP ON EXITλΪ1
	// SCB->SCR|=1<<1;//�Ĵ����汾������SLEEP ON EXITλΪ1
	__ASM volatile("wfi");
}

void App_RTCSleepTest(void)
{
	static UINT16 s_u16HaltTestCnt = 0;

	if (0 == g_st_SysTimeFlag.bits.b1Sys1000msFlag1)
	{ // ���������ȴ�ϵͳ��ʼ�����
		return;
	}

	if (++s_u16HaltTestCnt >= 5)
	{ // 10s����Test
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
		InitE2PROM();			//�������RTC�Ļ�����ֵ�Ķ�ȡ��EEPROM
		IOstatus_RTCMode();
		InitWakeUp_RTCMode();
		Sys_StopMode();
		//Sys_StandbyMode();
		IORecover_RTCMode();
		*/
	}
}
