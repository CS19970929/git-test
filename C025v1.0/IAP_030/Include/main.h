#ifndef __MAIN_H
#define __MAIN_H

//#define __EEPROM_TEST

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "I2C.h"
#include "SCI.h"
#include "stm32f0xx_it.h"			//里面有一些硬件错误之类的中断，还是需要的


//系统存储器：出厂由ST在这个区域内部预置了一段BootLoader，也就是ISP程序，这是一块ROM
//没有什么中低容量产品，只会用C8和R8，均为64K，	 每页1KB。系统存储器3KB。
#define FLASH_ADDR_IAP_START 			0x08000000		//IAP=7K，这个地方出了一次问题，修改后大于6K，侧面反映编译出来的Zi-data也是flash的东西
#define FLASH_ADDR_APP_START 			0x08001C00		//APP=64-7-1-1=55K
#define FLASH_ADDR_UPDATE_FLAG 			0x0800F800		//升级标志位，1K
#define FLASH_ADDR_SLEEP_FLAG           0x0800FC00		//休眠关键指令，1K

#define FLASH_TO_IAP_VALUE				((UINT16)0x00AB)
#define FLASH_TO_APP_VALUE				((UINT16)0xFFFF)

#define FLASH_NORMAL_SLEEP_VALUE    	((UINT16)0x1234)
#define FLASH_DEEP_SLEEP_VALUE    		((UINT16)0x1235)
#define FLASH_HICCUP_SLEEP_VALUE    	((UINT16)0x1236)
#define FLASH_SLEEP_RESET_VALUE    		((UINT16)0xFFFF)



union SYS_TIME{
    UINT8   all;
    struct StatusSysTimeFlagBit
    {
        UINT8 b1Sys10msFlag         :1;
        UINT8 b1Sys20msFlag         :1;
		UINT8 b1Sys200ms1Flag       :1;
		UINT8 b1Sys200msFlag        :1;
		
        UINT8 b1Sys10ms1Flag        :1;
        UINT8 b1Sys10ms2Flag        :1;
        UINT8 b1Sys10ms3Flag        :1;
        UINT8 b1Sys10ms4Flag        :1;
     }bits;
};

typedef struct _16_Bits_Struct {
    uint16_t bit0 : 1;
    uint16_t bit1 : 1;
    uint16_t bit2 : 1;
    uint16_t bit3 : 1;
    uint16_t bit4 : 1;
    uint16_t bit5 : 1;
    uint16_t bit6 : 1;
    uint16_t bit7 : 1;
    uint16_t bit8 : 1;
    uint16_t bit9 : 1;
    uint16_t bit10 : 1;
    uint16_t bit11 : 1;
    uint16_t bit12 : 1;
    uint16_t bit13 : 1;
    uint16_t bit14 : 1;
    uint16_t bit15 : 1;
} Bits_16_TypeDef;

#define PORT_OUT_GPIOA    ((Bits_16_TypeDef *)(&(GPIOA->ODR)))
#define PORT_OUT_GPIOB    ((Bits_16_TypeDef *)(&(GPIOB->ODR)))
#define PORT_OUT_GPIOC    ((Bits_16_TypeDef *)(&(GPIOC->ODR)))
#define PORT_OUT_GPIOF    ((Bits_16_TypeDef *)(&(GPIOF->ODR)))


#define MCUO_DEBUG_LED1 	(PORT_OUT_GPIOB->bit2)		//LED1
//#define MCUO_DEBUG_LED2 	PDout(3)		//PB3，LED2

//电源模块
#define MCUO_PWSV_STB 		(PORT_OUT_GPIOB->bit1)		//
#define MCUO_PWSV_LDO		(PORT_OUT_GPIOB->bit5)		//
#define MCUO_PWSV_CTR		(PORT_OUT_GPIOB->bit15)		//
//#define MCUO_BAT_CHG_EN 	PAout(11)					//RTC电池预，无

#define MCU_RESET()			NVIC_SystemReset()
#define UPGRATER_TIMEOUT 	25

#define _COMMOM_UPPER_SCI1
#define _COMMOM_UPPER_SCI2



extern uint8_t u8FlagUdFinishE2PROM;
extern uint8_t u8FlagUdFinish;

void __delay_ms(INT16 nms);

#endif /* __MAIN_H */
