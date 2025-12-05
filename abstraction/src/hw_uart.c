#include "hw_uart.h"

USART_Handle_t usart_init(USART_RegDef_t * UARTx, uint32_t BAUDRATE, uint8_t HW_FLOW_CTRL, uint8_t MODE, uint8_t STOP_BITS, uint8_t WORDLEN, uint8_t PARITY)
{
	static USART_Handle_t USARTx_handle;

	USARTx_handle.pUSARTx = UARTx;
	USARTx_handle.USART_Config.USART_Baud = BAUDRATE;
	USARTx_handle.USART_Config.USART_HWFlowControl = HW_FLOW_CTRL;
	USARTx_handle.USART_Config.USART_Mode = MODE;
	USARTx_handle.USART_Config.USART_NoOfStopBits = STOP_BITS;
	USARTx_handle.USART_Config.USART_WordLength = WORDLEN;
	USARTx_handle.USART_Config.USART_ParityControl = PARITY;
	USART_Init(&USARTx_handle);

	USART_PeripheralControl(UARTx,ENABLE);

	return USARTx_handle;
}

void USART_Send(USART_Handle_t * UARTx, uint8_t *TX_BUFFER, uint32_t LENGTH)
{
	USART_SendData(UARTx, TX_BUFFER, LENGTH);
}

void USART_Recieve(USART_Handle_t * UARTx, uint8_t *RX_BUFFER, uint32_t LENGTH)
{
	USART_ReceiveData(UARTx, RX_BUFFER, LENGTH);
}
