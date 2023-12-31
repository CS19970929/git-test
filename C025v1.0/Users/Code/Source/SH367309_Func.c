#include "main.h"

SH367309_REG_STORE SH367309_Reg_Store;

UINT8 gu8_DriverStartUpFlag = 0;

UINT16 iSheldTemp_10K_NTC[141] = {	20375,19204,18115,17100,16152,15266,14437,13661,12934,12251,
									11611,11008,10442,9909,9407,8935,8489,8068,7672,7297,
									6943,6608,6292,5993,5710,5442,5188,4948,4720,4504,
									4300,4105,3921,3746,3580,3422,3272,3130,2994,2866,
									2751,2627,2516,2410,2310,2214,2123,2036,1953,1874,
									1801,1726,1658,1592,1530,1470,1413,1358,1306,1256,
									1209,1163,1119,1078,1038,1000,963,928,894,862,
									831,801,773,746,719,694,670,647,625,604,
									583,563,544,526,509,492,476,460,445,431,
									416,403,390,378,366,355,343,333,322,312,
									303,294,285,276,268,260,252,244,237,230,
									224,217,211,205,199,193,188,182,177,172,
									167,163,158,154,150,146,142,138,134,131,
									127,124,120,117,114,111,108,106,103,100,
									98};


//前26个寄存器默认参数
UINT8 ucMTPBuffer[26] = {
    BYTE_00H_SCONF1,	BYTE_01H_SCONF2,		BYTE_02H_OVT_LDRT_OVH,	BYTE_03H_OVL,			BYTE_04H_UVT_OVRH,
	BYTE_05H_OVRL,		BYTE_06H_UV,			BYTE_07H_UVR,			BYTE_08H_BALV,			BYTE_09H_PREV,
	BYTE_0AH_L0V,		BYTE_0BH_PFV,			BYTE_0CH_OCD1V_OCD1T,	BYTE_0DH_OCD2V_OCD2T,	BYTE_0EH_SCV_SCT,
	BYTE_0FH_OCCV_OCCT,	BYTE_10H_MOST_OCRT_PFT,	BYTE_11H_OTC,			BYTE_12H_OTCR,			BYTE_13H_UTC,
	BYTE_14H_UTCR,		BYTE_15H_OTD,			BYTE_16H_OTDR,			BYTE_17H_UTD,			BYTE_18H_UTDR,
	BYTE_19H_TR
};


/*
摄氏度，避免被触发X+40
充电高温保护------->100
充电高温保护恢复--->99
充电低温保护------->-40
充电低温保护恢复--->-39
放电高温保护------->100
放电高温保护恢复--->99
放电低温保护------->-40
放电低温保护恢复--->-39
*/
UINT8 ucMTPBuffer_MCUDrivers[8] = {
	0x8C,
	0x8B,
	0x00,
	0x01,
	0x8C,
	0x8B,
	0x00,
	0x01,
};

/*******************************************************************************
Function:ResetAFE()
Description:  Reset SH367309 IC, Send Data:0xEA, 0xC0, CRC
Input:	 NULL
Output: NULL
Others:
*******************************************************************************/
void AFE_Reset(void)
{
	UINT8 WrBuf[2];

	WrBuf[0] = 0xC0;
	WrBuf[1] = 0xA5;

	/*
	if(!System_ErrFlag.u8ErrFlag_Com_AFE1) {
		if(!MTPWrite(AFE_ID, 0xEA, 1, WrBuf)) {              //0xEA, 0xC0?A CRC
			MTPWrite(AFE_ID, 0xEA, 1, WrBuf);
		}
		//MTPWrite(0xEA, 1, WrBuf);
	}
	*/

	if (!System_ERROR_UserCallback(ERROR_STATUS_AFE1))
	{
		if (!MTPWrite(0xEA, 1, WrBuf))
		{ // 0xEA, 0xC0?A CRC
			MTPWrite(0xEA, 1, WrBuf);
		}
	}
}

// 进入休眠模式
void AFE_Sleep(void)
{
	SH367309_Reg_Store.REG_MTP_CONF.bits.SLEEP = 1;
	MTPWrite(MTP_CONF, 1, &SH367309_Reg_Store.REG_MTP_CONF.all);
}

void AFE_IDLE(void)
{
	SH367309_Reg_Store.REG_MTP_CONF.bits.IDLE = 1;
	MTPWrite(MTP_CONF, 1, &SH367309_Reg_Store.REG_MTP_CONF.all);
}

// 进入休眠模式
void AFE_SHIP(void)
{
	MCUO_AFE_SHIP = 0;
}

// 进入IDLE模式
// 0，IDLE。1，Sleep。2，SHIP(没通讯上)
UINT8 AFE_SleepMode_Judge(void)
{
	UINT8 result = 0;

	MTPRead(MTP_BSTATUS1, 3, &SH367309_Reg_Store.REG_BSTATUS1.all);

	if (SH367309_Reg_Store.REG_BSTATUS1.bits.OV)
	{
		result = 2;
	}
	// 过放等进入普通休眠(和深度休眠差不多)
	else if (										/*g_stCellInfoReport.unMdlFault_Third.bits.b1BatUvp\
													|| g_stCellInfoReport.unMdlFault_Third.bits.b1CellUvp\
															*/
			 g_stCellInfoReport.u16VCellMin <= 2500 /*g_stCellInfoReport.u16VCellTotle <= 2500\*/
			 || System_ERROR_UserCallback(ERROR_STATUS_AFE1) || System_ERROR_UserCallback(ERROR_STATUS_AFE2) || System_ERROR_UserCallback(ERROR_STATUS_CBC_DSG))
	{ // 短路也整上去吧
		result = 1;
	}
	else
	{
		result = 0;
	}

	return result;
}

// 1，出问题，同时上报系统AFE错误
// 0，没问题
UINT8 AFE_IsReady(void)
{
	UINT8 TempCnt = 0, result = 0;
	UINT8 TempVar;

	while (1)
	{
		Feed_IWatchDog;

		TempVar = 0;
		if (MTPRead(MTP_BFLAG2, 1, &TempVar))
		{ // 读取BLFG2，查看VADC是否转换完成
			if ((TempVar & 0x10) == 0x10)
			{
				break;
			}
		}

		Delay1ms(20);
		if (++TempCnt >= 50)
		{
			// System_ERROR_UserCallback(ERROR_AFE1);
			result = 1;
			break;
		}
	}
	return result;
}

UINT8 AFE_GetData(void)
{
	UINT8 result;
	result = MTPRead(MTP_TEMP1, sizeof(Registers_AFE1), (UINT8 *)&Registers_AFE1);
	return result;
}

UINT32 AFE_CalcuVbat(void)
{
	UINT8 i;
	UINT32 result = 0;

	if (AFE_GetData())
	{
		for (i = 0; i < OtherElement.u16Sys_SeriesNum; i++)
		{
			result += ((UINT32)Registers_AFE1.Cell[i] * 5 >> 5); // Calculate a single battery voltage
																 // Vcell*5/32
		}
	}
	return result;
}

// 1：状态查询失败。0：成功
UINT8 AFE_CheckStatus(void)
{
	UINT8 result = 0;

	if (!MTPRead(MTP_BSTATUS1, 3, &SH367309_Reg_Store.REG_BSTATUS1.all))
	{
		result = 1;
	}
	return result;
}

/*******************************************************************************
Function:EnableAFEWdtCadc()
Description:使能CHG&DSG&PCHG输出，且使能WDT和CADC模块
Input:
Output:
Others:
*******************************************************************************/
void SH367309_Enable_AFE_Wdt_Cadc_Drivers(void)
{
	// ucMTP_CONF |= 0x04;						//开启看门狗，不开看门狗行不行
	// 结论，可以不开启。看门狗溢出，操作是
	// 1，关闭充放电MOS和预充MOS
	// 2，清除均衡
	// 两者对于目前使用情况意义不大，休眠带电不允许开，MCU控驱动没意义
	// 30x是需要开的，因为是自己的保护体系，这个309用的是他自己的体系，所以就算出问题
	// 看门狗不关，他自己的保护体系判断是否关MOS，风险也不大。
	SH367309_Reg_Store.REG_MTP_CONF.bits.CADCON = 1; // 开启CADC
	SH367309_Reg_Store.REG_MTP_CONF.bits.CHGMOS = 1; // 充电MOS由AFE硬件控制
	SH367309_Reg_Store.REG_MTP_CONF.bits.DSGMOS = 1; // 放电MOS由AFE硬件控制
	MTPWrite(MTP_CONF, 1, &SH367309_Reg_Store.REG_MTP_CONF.all);
}

// 1：修改失败。0：修改成功
UINT8 SH367309_SC_DelayT_Set(void)
{
	UINT8 result = 0;
	UINT8 u8temp_now = 0;
	UINT8 u8temp_need = 0;

	u8temp_now = SH367309_Reg_Store.u8_MTP_SCV_SCT & 0x0F;
	u8temp_need = OtherElement.u16CBC_DelayT >> 6; // 除以64得出等级，其表格就是以64us为一个等级的
	if (u8temp_need > 15)
		u8temp_need = 15;

	if (u8temp_now != u8temp_need)
	{
		if (MTPWriteROM(0x0E, 1, (UINT8 *)((SH367309_Reg_Store.u8_MTP_SCV_SCT & 0xF0) | u8temp_need)))
		{
			SH367309_Reg_Store.u8_MTP_SCV_SCT = (SH367309_Reg_Store.u8_MTP_SCV_SCT & 0xF0) | u8temp_need;
			OtherElement.u16CBC_DelayT = (UINT16)u8temp_need << 6;
		}
		else
		{
			// 写失败
			OtherElement.u16CBC_DelayT = (UINT16)u8temp_now << 6;
			result = 1;
		}
	}
	else
	{
		// 相同，则修改上传参数便可
		OtherElement.u16CBC_DelayT = (UINT16)u8temp_now << 6;
	}

	return result;
}

// 休眠带电需要的，控制权全权交给AFE
// MCU的驱动，完全不用管内部各种保护是怎么样，只需要把影响均衡的两个地方配置好就行
// 温度保护设置宽一些
// 取消二次过充保护
// 目前打算把两个函数合并，AFE驱动和MCU驱动区别，只要把温度数值替换就行，两者都是取消二次过充保护的
// 结果是，使用MCU驱动时，稍微影响一点点开机时间，问题不大
void SH367309_UpdataAfeConfig_Old(void)
{
	UINT8 bufferbak[26], mtpbufferbak[26];
	UINT8 i;
	UINT16 ResTemp[8];

	Feed_IWatchDog;

// 如果没有使用AFE驱动，只使用MCU驱动，则温度保护要设置宽一些
#if (!defined _SH367309_DRIVERS)
	for (i = 0; i < 8; ++i)
	{
		ucMTPBuffer[i + MTP_OTC] = ucMTPBuffer_MCUDrivers[i];
	}
#endif

	// 和EEPROM相关寄存器修改
	ucMTPBuffer[0x0E] = (BYTE_0EH_SCV_SCT & 0xF0) | (OtherElement.u16CBC_DelayT >> 6);

	SH367309_Reg_Store.u8_MTP_SCONF2 = ucMTPBuffer[0x01];
	SH367309_Reg_Store.u8_MTP_SCV_SCT = ucMTPBuffer[0x0E];

	if (MTPRead(0x00, 26, bufferbak))
	{													// 读309配置,包括TR
		ucMTPBuffer[MTP_TR] = bufferbak[MTP_TR] & 0x7F; // 先获取TR[6~0]的值
		MemoryCopy(ucMTPBuffer, mtpbufferbak, sizeof(ucMTPBuffer));
		SH367309_Reg_Store.TR_ResRef = 680 + 5 * ucMTPBuffer[MTP_TR];

		// 只要保证低温保护类型低于25摄氏度，(UINT8)强制转换相当于-256处理了
		for (i = 0; i < 8; ++i)
		{
			ResTemp[i] = iSheldTemp_10K_NTC[ucMTPBuffer[i + MTP_OTC]];
			mtpbufferbak[i + MTP_OTC] = (UINT8)(((UINT32)ResTemp[i] << 9) / ((UINT32)SH367309_Reg_Store.TR_ResRef + ResTemp[i]));
		}

		/*
		ResTemp[0] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_OTC]];
		ResTemp[1] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_OTCR]];
		ResTemp[2] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_UTC]];
		ResTemp[3] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_UTCR]];
		ResTemp[4] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_OTD]];
		ResTemp[5] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_OTDR]];
		ResTemp[6] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_UTD]];
		ResTemp[7] = iSheldTemp_10K_NTC[ucMTPBuffer[MTP_UTDR]];
		mtpbufferbak[MTP_OTC] = (UINT8)(((UINT32)ResTemp[0]<<9)/((UINT32)ResRef + ResTemp[0]));
		mtpbufferbak[MTP_OTCR] = (UINT8)(((UINT32)ResTemp[1]<<9)/((UINT32)ResRef + ResTemp[1]));
		mtpbufferbak[MTP_UTC] = (UINT8)(((UINT32)ResTemp[2]<<9)/((UINT32)ResRef + ResTemp[2]) - 256);
		mtpbufferbak[MTP_UTCR] = (UINT8)(((UINT32)ResTemp[3]<<9)/((UINT32)ResRef + ResTemp[3]) - 256);
		mtpbufferbak[MTP_OTD] = (UINT8)(((UINT32)ResTemp[4]<<9)/((UINT32)ResRef + ResTemp[4]));
		mtpbufferbak[MTP_OTDR] = (UINT8)(((UINT32)ResTemp[5]<<9)/((UINT32)ResRef + ResTemp[5]));
		mtpbufferbak[MTP_UTD] = (UINT8)(((UINT32)ResTemp[6]<<9)/((UINT32)ResRef + ResTemp[6]) - 256);
		mtpbufferbak[MTP_UTDR] = (UINT8)(((UINT32)ResTemp[7]<<9)/((UINT32)ResRef + ResTemp[7]) - 256);
		*/
		for (i = 0; i < 25; i++)
		{ // 最后一个TR不做对比
			if (bufferbak[i] != mtpbufferbak[i])
			{

				if (AFE_CalcuVbat() < 18000)
				{ // 如果AFE供电低于18V，烧写时VPRO电压可能不足8V，不能烧写
					System_ERROR_UserCallback(ERROR_AFE1);
					break;
				}

				MCUO_AFE_VPRO = 1;
				Delay1ms(20);
				if (!MTPWriteROM(0x00, 25, mtpbufferbak))
				{ // 重写EEPROM的寄存器，两次
					// System_ERROR_UserCallback(ERROR_AFE1);
				}
				MCUO_AFE_VPRO = 0;
				Delay1ms(1);

				if (!System_ERROR_UserCallback(ERROR_STATUS_AFE1))
				{
					// EA = 0;								//啥意思啊，这句话TODO
					AFE_Reset(); // Reset IC
					Delay1ms(5);
					AFE_IsReady();
				}

				break;
			}
		}

		if (!System_ERROR_UserCallback(ERROR_STATUS_AFE1))
		{
			if (MTPRead(0x00, 26, bufferbak))
			{ // Read MTP value
				for (i = 0; i < 25; i++)
				{ // 最后一个TR不做对比
					if (bufferbak[i] != mtpbufferbak[i])
					{
						System_ERROR_UserCallback(ERROR_AFE1);
						// break;
					}
				}
			}
		}
	}
}

void SH367309_DriverMos_Ctrl(GPIO_Type Type, UINT8 OnOFF)
{
	switch (Type)
	{
	case GPIO_PreCHG:
		SH367309_Reg_Store.REG_MTP_CONF.bits.PCHMOS = OnOFF;
		break;

	case GPIO_CHG:
		SH367309_Reg_Store.REG_MTP_CONF.bits.CHGMOS = OnOFF;
		break;

	case GPIO_DSG:
		SH367309_Reg_Store.REG_MTP_CONF.bits.DSGMOS = OnOFF;
		break;

	default:
		break;
	}

	MTPWrite(MTP_CONF, 1, &SH367309_Reg_Store.REG_MTP_CONF.all);
}

void SH367309_Driver_Supplement(void)
{
	static UINT8 su8_MOS_DSG_Status = 0;
	UINT8 DSG_Close_Flag = 0;
	static UINT32 su32_PreRelayOPEN_MODE_Cnt = 0;
	static UINT32 su32_Delay_Cnt = 0;
	static UINT8 su8_OpenT_Cnt = 0;

	if (0 == g_st_SysTimeFlag.bits.b1Sys10msFlag3)
	{
		return;
	}

	switch (su8_MOS_DSG_Status)
	{
	case 0:
		// 刚开机也是关闭，所以问题不大。
		// 运行期间检查到关闭，也要执行一遍循环
		if (!gu8_DriverStartUpFlag)
		{ // 把电流校准放在前面
			return;
		}

		if (SystemStatus.bits.b1Status_MOS_DSG == CLOSE)
		{
			MCUO_AFE_CTLC = 0; // 先强制关闭再说
			su8_MOS_DSG_Status = 1;
		}
		break;

	case 1:
		DSG_Close_Flag = 0; // 默认是不Close

		// 自身丰富添加的保护模式
		if (g_stCellInfoReport.unMdlFault_First.bits.b1IdischgOcp)
		{
			// 是扩展的放电保护关掉放电MOS
			// 保持不动
			DSG_Close_Flag = 1;
		}
		else if (SH367309_Reg_Store.REG_BSTATUS1.bits.UV)
		{
			DSG_Close_Flag = 1;
		}
		else if (SH367309_Reg_Store.REG_BSTATUS1.bits.OCD1)
		{
			DSG_Close_Flag = 1;
		}
		else if (SH367309_Reg_Store.REG_BSTATUS1.bits.OCD2)
		{
			DSG_Close_Flag = 1;
		}
		else if (SH367309_Reg_Store.REG_BSTATUS1.bits.SC)
		{
			DSG_Close_Flag = 1;
		}
		else if (SH367309_Reg_Store.REG_BSTATUS2.bits.UTD)
		{
			DSG_Close_Flag = 1;
		}
		else if (SH367309_Reg_Store.REG_BSTATUS2.bits.OTD)
		{
			DSG_Close_Flag = 1;
		}
		else if (SystemStatus.bits.b1Status_BnCloseIO)
		{ // 如果有别的手动关闭的信号，必须添加到这里
			DSG_Close_Flag = 1;
		}
		else
		{
			// 1，看门狗溢出--------会有AFE报错。不允许出现，出现必须改代码
			// 2，二次过充电保护----禁止(DIS_PF=1)
			// 3，断线检测----------禁止(DIS_PF=1)
		}

		if (DSG_Close_Flag == 0)
		{
			su8_MOS_DSG_Status = 2;
		}
		break;

	case 2:
		// 打开预充
		SystemStatus.bits.b1Status_MOS_PRE = !SystemStatus.bits.b1Status_MOS_PRE;
		if (SystemStatus.bits.b1Status_MOS_PRE == OPEN)
		{
			++su32_PreRelayOPEN_MODE_Cnt;
		}
		if (su32_PreRelayOPEN_MODE_Cnt >= (UINT32)OtherElement.u16Sys_PreChg_Time)
		{
			su32_PreRelayOPEN_MODE_Cnt = 0;
			su8_MOS_DSG_Status = 3;
		}
		break;

	case 3:
		MCUO_AFE_CTLC = 1; // 预充结束，允许打开放电MOS
		if (++su32_Delay_Cnt >= 10)
		{ // 延时100ms关闭预充
			su32_Delay_Cnt = 0;
			SystemStatus.bits.b1Status_MOS_PRE = CLOSE;
			su8_MOS_DSG_Status = 4;
		}
		break;

	case 4:
		if (SystemStatus.bits.b1Status_MOS_DSG == OPEN)
		{ // 监控是否真的打开了，打开才回到原来的地方监控。
			su8_MOS_DSG_Status = 0;
			su8_OpenT_Cnt = 0;
		}

		if (++su8_OpenT_Cnt >= 200)
		{ // 如果计时2s内没打开，则报错
			su8_OpenT_Cnt = 0;
			System_ERROR_UserCallback(ERROR_AFE2); // 说明没法打开，出问题
		}
		break;

	default:
		break;
	}

	MCUO_MOS_PRE = SystemStatus.bits.b1Status_MOS_PRE;
	// SystemStatus.bits.b1Status_Relay_PRE = MCUO_AFE_CTLC;
}

UINT16 Sys_FindProtectFilterMax_Real(void)
{
	UINT8 i;
	static UINT16 s_u16ProtectFilterMax = 0;
	if (0 != s_u16ProtectFilterMax)
	{
		return s_u16ProtectFilterMax + 2; // 推迟20ms
	}
	for (i = 0; i < 13; ++i)
	{
		if (*(&PRT_E2ROMParas.u16VcellOvp_Filter + 5 * i) > s_u16ProtectFilterMax)
		{
			s_u16ProtectFilterMax = *(&PRT_E2ROMParas.u16VcellOvp_Filter + 5 * i);
		}
	}
	return s_u16ProtectFilterMax + 2; // 推迟20ms
}

UINT16 aaaaaa1 = 0;
UINT16 aaaaaa2 = 0;
UINT16 aaaaaa3 = 0;
UINT16 aaaaaa4 = 0;
uint8_t aaa11 = 0;

UINT16 gu8_WakeUp_Type = 0;
void RTC_SleepMode_Ctrl(void)
{
	static UINT8 su8_StartUp_Flag = 0;
	static UINT16 su16_Delay_100msTCnt = 0;

	static UINT8 su8_SleepExtComCnt = 0;

	static UINT16 su16_RTC1_100msTCnt = 0;
	static UINT16 su16_Normal1_100msTCnt = 0;
	static UINT16 su16_RTC2_100msTCnt = 0;
	static UINT16 su16_Normal2_100msTCnt = 0;

	UINT8 su8_CurComDelay_Flag = 0;

	if (0 == g_st_SysTimeFlag.bits.b1Sys1000msFlag3)
	{
		return;
	}

	// gu8_WakeUp_Type = FLASH_VALUE_WAKE_RTC;

	switch (su8_StartUp_Flag)
	{
	case 0:
		if (++su16_Delay_100msTCnt >= (Sys_FindProtectFilterMax_Real() / 100 + 1))
		{
			su16_Delay_100msTCnt = 0;
			su8_StartUp_Flag = 1;
		}
		break;

	case 1:
		if ((g_stCellInfoReport.u16Ichg > 2) || (g_stCellInfoReport.u16IDischg > 2))
		{
			su8_CurComDelay_Flag = 1;
		}
#if 1
		else if (su8_SleepExtComCnt != RTC_ExtComCnt)
		{
			su8_SleepExtComCnt = RTC_ExtComCnt;
			su8_CurComDelay_Flag = 1;
		}
#endif

		if (su8_CurComDelay_Flag)
		{
			su16_RTC1_100msTCnt = 0;
			su16_RTC2_100msTCnt = 0;
			su16_Normal1_100msTCnt = 0;
			su16_Normal2_100msTCnt = 0;
			gu8_WakeUp_Type = FLASH_VALUE_WAKE_OTHER;
		}
		else
		{
			switch (gu8_WakeUp_Type)
			{
			case FLASH_VALUE_WAKE_RTC:
				if (AFE_SleepMode_Judge() == 1)
				{ // mos过温也纳入处理
					if (++su16_Normal1_100msTCnt > 6)
					{
						// if(++su16_Normal1_100msTCnt > 60*10) {
						su16_Normal1_100msTCnt = 0;
						Sleep_Mode.bits.b1ForceToSleep_L2 = 1;
					}
					if (su16_RTC1_100msTCnt)
						su16_RTC1_100msTCnt = 0;
				}
				else if (AFE_SleepMode_Judge() == 0)
				{
					if (++su16_RTC1_100msTCnt > 3)
					{
						su16_RTC1_100msTCnt = 0;
						Sleep_Mode.bits.b1ForceToSleep_L1 = 1;
					}
					if (su16_Normal1_100msTCnt)
						su16_Normal1_100msTCnt = 0;
				}
				else
				{
					// 不进入休眠
				}
				aaa11 = 1;
				break;

			case FLASH_VALUE_WAKE_OTHER:
				if (AFE_SleepMode_Judge() == 1)
				{
					if (++su16_Normal2_100msTCnt >= 6)
					{
						// if(++su16_Normal2_100msTCnt >= 60*10) {
						su16_Normal2_100msTCnt = 0;
						Sleep_Mode.bits.b1ForceToSleep_L2 = 1;
					}
					if (su16_RTC2_100msTCnt)
						su16_RTC2_100msTCnt = 0;
				}
				else if (AFE_SleepMode_Judge() == 0)
				{
					if (++su16_RTC2_100msTCnt >= 6)
					{
						// if(++su16_RTC2_100msTCnt >= 60*10) {
						su16_RTC2_100msTCnt = 0;
						Sleep_Mode.bits.b1ForceToSleep_L1 = 1;
					}
					if (su16_Normal2_100msTCnt)
						su16_Normal2_100msTCnt = 0;
				}
				else
				{
					// 不进入休眠
				}

				aaa11 = 2;
				break;

			default:
				gu8_WakeUp_Type = FLASH_VALUE_WAKE_OTHER;
				FlashWriteOneHalfWord(FLASH_ADDR_WAKE_TYPE, FLASH_VALUE_WAKE_OTHER);
				break;
			}
		}
		break;

	default:
		break;
	}

	if (Sleep_Mode.bits.b1ForceToSleep_L1)
	{
		// 如果是低压，则会自动进入L2，和L3也无区别
	}

	aaaaaa1 = su16_RTC1_100msTCnt;
	aaaaaa2 = su16_Normal1_100msTCnt;
	aaaaaa3 = su16_RTC2_100msTCnt;
	aaaaaa4 = su16_Normal2_100msTCnt;
}

void Fault_ChangeToMCU(void)
{
	static UINT8 su8_CellOvp_Flag = 0;
	static UINT8 su8_CellUvp_Flag = 0;
	static UINT8 su8_IdischgOcp1_Flag = 0;
	static UINT8 su8_IdischgOcp2_Flag = 0;
	static UINT8 su8_IchgOcp_Flag = 0;
	static UINT8 su8_CellChgUtp_Flag = 0;
	static UINT8 su8_CellChgOtp_Flag = 0;
	static UINT8 su8_CellDsgUtp_Flag = 0;
	static UINT8 su8_CellDsgOtp_Flag = 0;

	g_stCellInfoReport.unMdlFault_Third.bits.b1CellOvp = SH367309_Reg_Store.REG_BSTATUS1.bits.OV;
	switch (su8_CellOvp_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1CellOvp)
		{
			FaultWarnRecord2(CellOvp_Third);
			su8_CellOvp_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1CellOvp)
		{
			su8_CellOvp_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Third.bits.b1CellUvp = SH367309_Reg_Store.REG_BSTATUS1.bits.UV;
	switch (su8_CellUvp_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1CellUvp)
		{
			FaultWarnRecord2(CellUvp_Third);
			su8_CellUvp_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1CellUvp)
		{
			su8_CellUvp_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Second.bits.b1IdischgOcp = SH367309_Reg_Store.REG_BSTATUS1.bits.OCD1;
	switch (su8_IdischgOcp1_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Second.bits.b1IdischgOcp)
		{
			FaultWarnRecord2(IdischgOcp_Second);
			su8_IdischgOcp1_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Second.bits.b1IdischgOcp)
		{
			su8_IdischgOcp1_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Third.bits.b1IdischgOcp = SH367309_Reg_Store.REG_BSTATUS1.bits.OCD2;
	switch (su8_IdischgOcp2_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1IdischgOcp)
		{
			FaultWarnRecord2(IdischgOcp_Third);
			su8_IdischgOcp2_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1IdischgOcp)
		{
			su8_IdischgOcp2_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Third.bits.b1IchgOcp = SH367309_Reg_Store.REG_BSTATUS1.bits.OCC;
	switch (su8_IchgOcp_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1IchgOcp)
		{
			FaultWarnRecord2(IchgOcp_Third);
			su8_IchgOcp_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1IchgOcp)
		{
			su8_IchgOcp_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Third.bits.b1CellChgUtp = SH367309_Reg_Store.REG_BSTATUS2.bits.UTC;
	switch (su8_CellChgUtp_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1CellChgUtp)
		{
			FaultWarnRecord2(CellChgUTp_Third);
			su8_CellChgUtp_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1CellChgUtp)
		{
			su8_CellChgUtp_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Third.bits.b1CellChgOtp = SH367309_Reg_Store.REG_BSTATUS2.bits.OTC;
	switch (su8_CellChgOtp_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1CellChgOtp)
		{
			FaultWarnRecord2(CellChgOTp_Third);
			su8_CellChgOtp_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1CellChgOtp)
		{
			su8_CellChgOtp_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Third.bits.b1CellDischgUtp = SH367309_Reg_Store.REG_BSTATUS2.bits.UTD;
	switch (su8_CellDsgUtp_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1CellDischgUtp)
		{
			FaultWarnRecord2(CellDsgUTp_Third);
			su8_CellDsgUtp_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1CellDischgUtp)
		{
			su8_CellDsgUtp_Flag = 0;
		}
		break;

	default:
		break;
	}

	g_stCellInfoReport.unMdlFault_Third.bits.b1CellDischgOtp = SH367309_Reg_Store.REG_BSTATUS2.bits.OTD;
	switch (su8_CellDsgOtp_Flag)
	{
	case 0:
		if (g_stCellInfoReport.unMdlFault_Third.bits.b1CellDischgOtp)
		{
			FaultWarnRecord2(CellDsgOTp_Third);
			su8_CellDsgOtp_Flag = 1;
		}
		break;

	case 1:
		if (!g_stCellInfoReport.unMdlFault_Third.bits.b1CellDischgOtp)
		{
			su8_CellDsgOtp_Flag = 0;
		}
		break;

	default:
		break;
	}
}

void TemperatureCheck(void)
{
	// SH367309_Reg_Store.REG_BSTATUS2.bits.OTC;
	// SH367309_Reg_Store.REG_BSTATUS2.bits.UTC;
	// SH367309_Reg_Store.REG_BSTATUS2.bits.OTD;
	// SH367309_Reg_Store.REG_BSTATUS2.bits.UTD;

	if (SH367309_Reg_Store.REG_BSTATUS2.bits.OTC || SH367309_Reg_Store.REG_BSTATUS2.bits.UTC)
	{
		if (g_stCellInfoReport.u16Ichg == 0)
		{
			SH367309_Reg_Store.REG_BSTATUS2.bits.OTC = 0;
			SH367309_Reg_Store.REG_BSTATUS2.bits.UTC = 0;
		}
	}

	if (SH367309_Reg_Store.REG_BSTATUS2.bits.OTD || SH367309_Reg_Store.REG_BSTATUS2.bits.UTD)
	{
		if (g_stCellInfoReport.u16IDischg == 0)
		{
			SH367309_Reg_Store.REG_BSTATUS2.bits.OTD = 0;
			SH367309_Reg_Store.REG_BSTATUS2.bits.UTD = 0;
		}
	}
}

// mos控制汇总，历史保护记录加入体系
void App_SH367309_Monitor(void)
{
	static UINT8 su8_SC_Flag = 0;
	static UINT8 su8_EEPR_WR_Flag = 0;

	static UINT8 su8_CtrlMos_Flag = 0;

	if (0 == g_st_SysTimeFlag.bits.b1Sys100msFlag)
	{ // 这个时基不能随便调，影响MOS动作，初始化电流校准
		return;
	}

	// if(MTPRead(MTP_BSTATUS1, 3, &SH367309_Reg_Store.REG_BSTATUS1.all)) {
	if (MTPRead(MTP_BALANCEH, 5, &SH367309_Reg_Store.u8_MTP_BALANCEH))
	{
		// g_stCellInfoReport.u16BalanceFlag1 = SH367309_Reg_Store.u8_MTP_BALANCEL;
		// g_stCellInfoReport.u16BalanceFlag2 = SH367309_Reg_Store.u8_MTP_BALANCEH;
		// SystemStatus.bits.b1Status_MOS_PRE = SH367309_Reg_Store.REG_BSTATUS3.bits.PCHG_FET;
		SystemStatus.bits.b1Status_MOS_CHG = SH367309_Reg_Store.REG_BSTATUS3.bits.CHG_FET;
		SystemStatus.bits.b1Status_MOS_DSG = SH367309_Reg_Store.REG_BSTATUS3.bits.DSG_FET;

		TemperatureCheck();
		// 9个保护？
		Fault_ChangeToMCU();

		switch (su8_SC_Flag)
		{
		case 0:
			if (SH367309_Reg_Store.REG_BSTATUS1.bits.SC)
			{
				System_ERROR_UserCallback(ERROR_CBC_DSG);
				su8_SC_Flag = 1;
			}
			break;

		case 1:
			// 负载断开(DSGD管教电平低于VDSGD)，持续时间超过负载释放延时tD1，现在设置为2s
			if (!SH367309_Reg_Store.REG_BSTATUS1.bits.SC)
			{
				su8_SC_Flag = 0;
			}
			break;

		default:
			break;
		}

		// 观察类型，不能被置位，置位说明有问题，配置不对
		// SH367309_Reg_Store.REG_BSTATUS1.bits.PF;
		// SH367309_Reg_Store.REG_BSTATUS1.bits.WDT;
		// SH367309_Reg_Store.REG_BSTATUS3.bits.L0V;
		// SH367309_Reg_Store.REG_BSTATUS3.bits.EEPR_WR;
		switch (su8_EEPR_WR_Flag)
		{
		case 0:
			if (SH367309_Reg_Store.REG_BSTATUS3.bits.EEPR_WR)
			{
				System_ERROR_UserCallback(ERROR_EEPROM_STORE);
				su8_EEPR_WR_Flag = 1;
			}
			break;

		case 1:
			// 负载断开(DSGD管教电平低于VDSGD)，持续时间超过负载释放延时tD1，现在设置为2s
			if (!SH367309_Reg_Store.REG_BSTATUS3.bits.EEPR_WR)
			{
				su8_EEPR_WR_Flag = 0;
			}
			break;

		default:
			break;
		}

		if (SH367309_Reg_Store.REG_BSTATUS1.bits.PF)
		{
			System_ERROR_UserCallback(ERROR_SPI);
		}

		if (SH367309_Reg_Store.REG_BSTATUS1.bits.WDT)
		{
			System_ERROR_UserCallback(ERROR_UPPER);
		}

		if (SH367309_Reg_Store.REG_BSTATUS3.bits.L0V)
		{
			System_ERROR_UserCallback(ERROR_CLIENT);
		}

		// 原则上执行一次便可，不能根据MOS状态长期执行这个函数，不然效率低下
		switch (su8_CtrlMos_Flag)
		{
		case 0:
			if (SystemStatus.bits.b1Status_BnCloseIO)
			{
				SH367309_DriverMos_Ctrl(GPIO_CHG, 0);
				SH367309_DriverMos_Ctrl(GPIO_DSG, 0);
				su8_CtrlMos_Flag = 1;
			}
			break;

		case 1:
			if (!SystemStatus.bits.b1Status_BnCloseIO)
			{
				SH367309_DriverMos_Ctrl(GPIO_CHG, 1);
				SH367309_DriverMos_Ctrl(GPIO_DSG, 1);
				su8_CtrlMos_Flag = 0;
			}
			break;

		default:
			break;
		}
	}
	else
	{
		// 读取失败，要做点什么事情？
		// 进入深度休眠咯？
		// Sleep_Mode.bits.b1ForceToSleep_L3 = 1;
	}
}

void App_DI1_Switch(void)
{
	if (0 == g_st_SysTimeFlag.bits.b1Sys200msFlag3)
	{
		return;
	}

#ifdef _DI_SWITCH_DSG_ONOFF
	static uint8_t su8_OnOFF_Flag = 0;
	static uint8_t su8_Repeat_Tcnt = 0;

	switch (su8_OnOFF_Flag)
	{
	case 0:
		if (MCUI_ENI_DI1)
		{
			SH367309_DriverMos_Ctrl(GPIO_DSG, 0);
			if (++su8_Repeat_Tcnt >= 5)
			{
				su8_Repeat_Tcnt = 0;
				su8_OnOFF_Flag = 1;
			}
		}
		else
		{
			su8_Repeat_Tcnt = 0;
			su8_OnOFF_Flag = 1;
		}
		break;

	case 1:
		if (!MCUI_ENI_DI1)
		{
			SH367309_DriverMos_Ctrl(GPIO_DSG, 1);
			if (++su8_Repeat_Tcnt >= 5)
			{
				su8_Repeat_Tcnt = 0;
				su8_OnOFF_Flag = 0;
			}
		}
		else
		{
			su8_Repeat_Tcnt = 0;
			su8_OnOFF_Flag = 0;
		}
		break;

	default:
		break;
	}
#endif

#ifdef _DI_SWITCH_SYS_ONOFF
	static UINT16 su16_AntiShake_Cnt1 = 0;

	if (1 == MCUI_ENI_DI1)
	{
		if (++su16_AntiShake_Cnt1 >= 2)
		{
			su16_AntiShake_Cnt1 = 2;
			Sleep_Mode.bits.b1ForceToSleep_L2 = 1;
		}
	}
	else
	{
		if (su16_AntiShake_Cnt1)
		{
			--su16_AntiShake_Cnt1;
			return;
		}
		// SleepElement.Sleep_Mode.bits.b1ForceToSleep_L2 = 0;
	}
#endif
}

void App_SH367309(void)
{
	App_SH367309_Monitor(); // 100ms时基
	App_SH367309_Supplement();
	SH367309_UpdataAfeConfig();
	App_DI1_Switch();

#ifdef _SLEEP_WITH_CURRENT
	RTC_SleepMode_Ctrl();
#else
	SH367309_Driver_Supplement(); // 10ms时基，需要App_SH367309_Monitor()读回来的内容			   //休眠带电目前不需要预充。
#endif
}
