#ifndef UART_CLIENT_H
#define UART_CLIENT_H

#define USART_TX_BUF_LEN 			100

#define	UART_CLIENT_HEAD_ADDR		((UINT8)0x5A)
#define	UART_CLIENT_END_VALUE		((UINT8)0xFB)


enum UART_CLIENT_CMD_E {
	UART_CLIENT_CMD_0x01 = 0xA1,
	UART_CLIENT_CMD_0x02 = 0xA2,
};


extern struct RS485MSG g_UartClientMsgPtr;
extern UINT16 g_u16UartClientCommuErrCnt;

void UartClient_FaultChk(void);
void UartClient_Rx_Deal(struct RS485MSG *s);
void InitUSART_UartClient(void);
void App_UartClient_Updata(struct RS485MSG *s);


void Sci_Rx_Deal_Client(struct RS485MSG *s);

#endif	/* UART_CLIENT_H */
