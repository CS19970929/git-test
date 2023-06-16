#ifndef ADC_H
#define ADC_H

#define AD_Used_amount 1

#define AD_CalNum			8		//后面是位移处理>>2，所以这个别乱改
#define AD_CalNum_2			3		//2^3 = 8，上面那个数是2的多少次方，用于位移

#define AD_CalNum_Cur		32		//后面是位移处理>>5，所以这个别乱改
#define AD_CalNum_Cur_2		5		//2^5 = 32，上面那个数是2的多少次方，用于位移



//AD采样变量枚举
enum tagInfoForADCArray
{
    ADC_TEMP_MOS1,                    	// MOS2温度
    ADC_TEMP_EV,                     	// 外部温度
    ADC_VBC,                         	// 母线电压
    //ADC_TEMP_MOS2,                    	// MOS1温度
    //ADC_CURR,                         	// 电流
    //ADC_EXT_C1,                       	// 第1节电池电压
    //ADC_EXT_C2,                       	// 第2节电池电压

	ADC_NUM		                            // ADC number
};


#define Vbc_scale_16 	  31	  		//总压采比例值
#define Vbc_scale_6 	  11	  		//总压采比例值


extern INT32 g_i32ADCResult[ADC_NUM];             //ADC数据缓存


void InitADC(void);
void App_AnlogCal(void);

#endif	/* ADC_H */
