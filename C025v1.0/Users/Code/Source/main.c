#include "main.h"

UINT8 SeriesNum = 16;

const unsigned char SeriesSelect_AFE1[16][16] = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 1串
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},	   // 2串
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
	InitDevice(); // 初始化外设，这两个函数的位置需要斟酌一下，现在换回去先
	InitVar();	  // 初始化变量

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
		App_SleepDeal(); // 放在App_MOS_Relay_Control()后面
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

// 这个初始化函数很容易出问题
void InitDevice(void)
{
	SystemInit(); // 直接调用就可以了。
				  // A，先reset所有配置，使用HSI(8MHz)运行。reset默认是使用HSI运行。
				  // B，调用SetSysClock()，默认使用8MHz外部晶振，然后六倍频输出，倍频输出不能超过48MHz(我使用12MHz，所以改为4倍频)
				  // C，如果倍频失败，会有个else语句让我改，输出一些标志位，我目前没改
				  // D，当从待机和停止模式返回或用作系统时钟的HSE 振荡器发生故障时，该位由硬件置来启动HSI 振荡器。
				  // E，言下之意，进入待机模式要关外部晶振，回来，先用HSI运行，然后开启外部晶振和倍频。
				  // F，还有一个切换时钟的函数，SystemCoreClockUpdate()，使用条件后面了解。
				  // G，外部晶振修改的话，改主头文件HSE_VALUE的值，会影响串口波特率。
				  // H，如果不使用HSE，直接焊掉外部晶振便可，系统会默认返回HSI，SystemCoreClock自动改为8M，后续观察串口波特率和I2C频率是否符合需求
	Init_IAPAPP();

#if (defined _DEBUG_CODE)
	IsSleepStartUp();
	InitIO();
	InitDelay();
	InitTimer();
	InitSystemWakeUp();
	InitE2PROM(); // 内部EEPROM，不需要初始化
	Init_RTC();

#else
	IsSleepStartUp();
	InitIO();
	InitDelay();
	InitTimer();
	InitSystemWakeUp();
	InitE2PROM(); // 内部EEPROM，不需要初始化
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

	// Switch功能
	// Switch_OnOFF_Func.all = 0;
	// InitSwitchData_EEPROM();

	// 总系统错误监控系统初始化
	for (i = 0; i < ERROR_NUM; ++i)
	{
		//*(&System_ErrFlag.u8ErrFlag_Com_AFE1+i)  =  0;		//有病，这样写就把错误清除了
	}

	// 保护标志位初始化
	g_stCellInfoReport.unMdlFault_First.all = 0;
	g_stCellInfoReport.unMdlFault_Second.all = 0;
	g_stCellInfoReport.unMdlFault_Third.all = 0;
	// 当次保护记录初始化
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

	// 继电器驱动开启初始化							//不打算放在这里
	// RelayCtrl_Command = RELAY_PRE_DET;	//不打算放在这里
	// HeatCtrl_Command = ST_HEAT_DET_SELF;
	// CoolCtrl_Command = ST_COOL_DET_SELF;

	// 休眠相关

	// 这样写就不用管前面到底读出来还是复位了(在EEPROM很多个地方算)
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

	MCUO_AFE_CTLC = 1; // 刚上电，默认高阻态，所以不慌AFE刚开机瞬间打开MOS
	MCUO_AFE_SHIP = 1;
	MCUO_AFE_MODE = 1;

	__delay_ms(100);
}

