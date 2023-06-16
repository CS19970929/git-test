#ifndef SCI_H
#define SCI_H

#define	SCI_TX_BUF_LEN			130   //原来130和100
#define RS485_MAX_BUFFER_SIZE 	1200

#define	RS485_BROADCAST_ADDR		(( uint8_t ) 0x00 )
#define	RS485_SLAVE_ADDR			(( uint8_t ) 0x01 )

//RS485状态机状态
#define	RS485_STA_IDLE				0
#define	RS485_STA_RX_COMPLETE		1
#define	RS485_STA_RX_OK				2
#define	RS485_STA_TX_COMPLETE		3

#define RS485_ADDR_RO_START				0xD000


#define	RS485_ACK_POS			        0x00	// 正响应
#define	RS485_ACK_NEG			        0x01	// 负响应
//Error type
#define	RS485_ERROR_ADDR_INVALID	    0x01	// 地址不合法
#define	RS485_ERROR_CRC_ERROR			0x02	// CRC校验错误
#define	RS485_ERROR_DATA_INVALID	    0x03	// 参数不合法
#define	RS485_ERROR_CMD_INVALID			0x04	// 当前状态下命令无效
#define	RS485_ERROR_RONLY_NO_W			0x05	// 只读参数拒绝写入
#define	RS485_ERROR_WONLY_NO_R			0x06	// 只写参数拒绝读取
#define	RS485_ERROR_NO_PERMISSION		0x07	// 无权限
#define	RS485_ERROR_NULL			    0x08	// 未知错误

#define RS485_CMD_ADDR_FLASH_CONNECT		0xFFFD	//MCU连接检测
//#define RS485_CMD_ADDR_FLASH_SIZE			0xFFFE	//MCU HEX文件大小
#define RS485_CMD_ADDR_FLASH_UPGRATE		0xFFFE	//代码升级命令
#define RS485_CMD_ADDR_FLASH_COMPLETE		0xFFFF	//代码升级命令

//485 cmd type
enum RS485_CMD_E {
	RS485_CMD_READ_REGS = 3,
	RS485_CMD_WRITE_REG = 6,
	RS485_CMD_WRITE_REGS = 16
};


extern struct RS485MSG g_stSCI1CurrentMsgPtr;
extern uint8_t u8FlashReceiveCnt;
extern struct RS485MSG g_stSCI2CurrentMsgPtr;


void InitUSART1_CommonUpper(void);
void App_CommonUpper_Sci1(struct RS485MSG *s);

void InitUSART2_CommonUpper(void);
void App_CommonUpper_Sci2(struct RS485MSG *s);

FLASH_Status FlashWriteOneHalfWord(uint32_t StartAddr,uint16_t Buffer);
UINT16 FlashReadOneHalfWord(UINT32 faddr);

#endif
