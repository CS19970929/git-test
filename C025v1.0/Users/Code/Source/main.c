#include "main.h"

UINT8 SeriesNum = 16;

const unsigned char SeriesSelect_AFE1[16][16] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 1��
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 2��
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 3
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 4
	{0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 5
	{0, 1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 6
	{0, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 7
	{0, 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0},	   // 8
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0},	   // 9
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0},	   // 10
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 0, 0, 0, 0},	   // 11
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 0, 0, 0},	   // 12
	{0, 1, 2, 3, 4, 5, 6, 7, 9, 9, 10, 11, 12, 0, 0, 0},   // 13
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 0},  // 14
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0}, // 15
	{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15} // 16
};

void InitVar(void);
void InitDevice(void);
void InitSystemWakeUp(void);
void App_MOS_Relay_Control(void);

int main(void)
{
	InitDevice(); // ��ʼ�����裬������������λ����Ҫ����һ�£����ڻ���ȥ��
	InitVar();	  // ��ʼ������

	while (1)
	{
#if (defined _DEBUG_CODE)
		App_SysTime();
		App_RTCSleepTest();
		App_SleepDeal();
		Feed_IWatchDog;

#else
		App_SysTime();
		App_CommonUpper();
		App_AFEGet();
		App_SH367309();
		App_AnlogCal();

		App_E2promDeal();
		App_SleepDeal(); // ����App_MOS_Relay_Control()����
		App_SOC();
		App_CellBalance();
		App_WarnCtrl();
		App_ChargerLoad_Det();
		App_Heat_Cool_Ctrl();

		App_FlashUpdateDet();
		App_LogRecord();
		App_ProID_Deal();

		Feed_IWatchDog;
#endif
	}
}

// �����ʼ�����������׳�����
void InitDevice(void)
{
	SystemInit(); // ֱ�ӵ��þͿ����ˡ�
				  // A����reset�������ã�ʹ��HSI(8MHz)���С�resetĬ����ʹ��HSI���С�
				  // B������SetSysClock()��Ĭ��ʹ��8MHz�ⲿ����Ȼ������Ƶ�������Ƶ������ܳ���48MHz(��ʹ��12MHz�����Ը�Ϊ4��Ƶ)
				  // C�������Ƶʧ�ܣ����и�else������Ҹģ����һЩ��־λ����Ŀǰû��
				  // D�����Ӵ�����ֹͣģʽ���ػ�����ϵͳʱ�ӵ�HSE ������������ʱ����λ��Ӳ����������HSI ������
				  // E������֮�⣬�������ģʽҪ���ⲿ���񣬻���������HSI���У�Ȼ�����ⲿ����ͱ�Ƶ��
				  // F������һ���л�ʱ�ӵĺ�����SystemCoreClockUpdate()��ʹ�����������˽⡣
				  // G���ⲿ�����޸ĵĻ�������ͷ�ļ�HSE_VALUE��ֵ����Ӱ�촮�ڲ����ʡ�
				  // H�������ʹ��HSE��ֱ�Ӻ����ⲿ�����ɣ�ϵͳ��Ĭ�Ϸ���HSI��SystemCoreClock�Զ���Ϊ8M�������۲촮�ڲ����ʺ�I2CƵ���Ƿ��������
	Init_IAPAPP();

#if (defined _DEBUG_CODE)
	IsSleepStartUp();
	InitIO();
	InitDelay();
	InitTimer();
	InitSystemWakeUp();
	InitE2PROM(); // �ڲ�EEPROM������Ҫ��ʼ��
	Init_RTC();

#else
	IsSleepStartUp();
	InitIO();
	InitDelay();
	InitTimer();
	InitSystemWakeUp();
	InitE2PROM(); // �ڲ�EEPROM������Ҫ��ʼ��
	InitUSART_CommonUpper();
	InitADC();
	InitData_SOC();
	Init_ChargerLoad_Det();
	InitHeat_Cool();
	InitAFE1();

// Init_IWDG();
#endif
}

void InitVar(void)
{
	UINT16 i;

	InitSystemMonitorData_EEPROM();
	SystemStatus.bits.b1Status_MOS_DSG = CLOSE;
	SystemStatus.bits.b1StartUpBMS = 0;

	// Switch����
	// Switch_OnOFF_Func.all = 0;
	// InitSwitchData_EEPROM();

	// ��ϵͳ������ϵͳ��ʼ��
	for (i = 0; i < ERROR_NUM; ++i)
	{
		//*(&System_ErrFlag.u8ErrFlag_Com_AFE1+i)  =  0;		//�в�������д�ͰѴ��������
	}

	// ������־λ��ʼ��
	g_stCellInfoReport.unMdlFault_First.all = 0;
	g_stCellInfoReport.unMdlFault_Second.all = 0;
	g_stCellInfoReport.unMdlFault_Third.all = 0;
	// ���α�����¼��ʼ��
	FaultPoint_First = 0;
	FaultPoint_Second = 0;
	FaultPoint_Third = 0;

	FaultPoint_First2 = 0;
	FaultPoint_Second2 = 0;
	FaultPoint_Third2 = 0;
	for (i = 0; i < Record_len; ++i)
	{
		Fault_record_First[i] = 0;
		Fault_record_Second[i] = 0;
		Fault_record_Third[i] = 0;

		Fault_record_First2[i] = 0;
		Fault_record_Second2[i] = 0;
		Fault_record_Third2[i] = 0;
	}
	Fault_Flag_Fisrt.all = 0;
	Fault_Flag_Second.all = 0;
	Fault_Flag_Third.all = 0;

	// �̵�������������ʼ��							//�������������
	// RelayCtrl_Command = RELAY_PRE_DET;	//�������������
	// HeatCtrl_Command = ST_HEAT_DET_SELF;
	// CoolCtrl_Command = ST_COOL_DET_SELF;

	// �������

	// ����д�Ͳ��ù�ǰ�浽�׶��������Ǹ�λ��(��EEPROM�ܶ���ط���)
	SeriesNum = OtherElement.u16Sys_SeriesNum;
	g_u32CS_Res_AFE = ((UINT32)OtherElement.u16Sys_CS_Res_Num * 1000) / OtherElement.u16Sys_CS_Res;

	SystemStatus.bits.b4Status_ProjectVer = 1;
	LogRecord_Flag.bits.Log_StartUp = 1;
}

void App_WakeUpAFE(void)
{
}

UINT8 App_AFEshutdown(void)
{
	return 0;
}

void InitSystemWakeUp(void)
{
	// MCUO_SD_DRV_CHG = 0;

	MCUO_PWSV_STB = 0;

	// MCUO_PWSV_LDO = 1;
	MCUO_PWSV_CTR = 1;

	// MCUO_DRV_WLM_PW = 1;

	MCUO_AFE_CTLC = 1; // ���ϵ磬Ĭ�ϸ���̬�����Բ���AFE�տ���˲���MOS
	MCUO_AFE_SHIP = 1;
	MCUO_AFE_MODE = 1;

	__delay_ms(100);
}

