/*
 * Uart_frame.c
 *
 *  Created on: 2 Oca 2026
 *      Author: busn
 */
#include <UartComm.h>

void Uart_Init(volatile UartComm_t *port, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx){
	port->huart		= huart;
	port->hdma_rx	= hdma_rx;
	port->rx_len	= 0;
	port->tx_len	= 0;
	port->rx_ready	= 0;
	port->tx_ready	= 0;
	port->tx_busy 	= 0;

	memset(port->rx_buff, 0, UART_BUFFER_SIZE);
	memset(port->tx_buff, 0, UART_BUFFER_SIZE);
	/* Son baytı '\0' için rezerve ederek DMA'yı başlat */
	HAL_UART_Receive_DMA(port->huart, port->rx_buff, UART_BUFFER_SIZE-1);
	__HAL_UART_ENABLE_IT(port->huart, UART_IT_IDLE);
}

void Uart_HandleIdleInterrupt(volatile UartComm_t *port){
	if(__HAL_UART_GET_FLAG(port->huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(port->huart);
		HAL_UART_DMAStop(port->huart);
		uint16_t remaining = __HAL_DMA_GET_COUNTER(port->hdma_rx);
		uint16_t len = (UART_BUFFER_SIZE -1) - remaining;
		if(len > 0 && len < UART_BUFFER_SIZE)
		{
			port->rx_buff[len] = '\0';
			port->rx_len = len;
			port->rx_ready = 1;
		}
		else
		{
			port->rx_buff[UART_BUFFER_SIZE-1] = '\0';
		}

		HAL_UART_Receive_DMA(port->huart, port->rx_buff, UART_BUFFER_SIZE-1);
	}
}

void Uart_SendMsg(volatile UartComm_t *port, const char *str)
{
	uint32_t tickstart = HAL_GetTick();
	while(port->tx_busy)
	{
		if((HAL_GetTick() - tickstart) > 10)
		{
			port->tx_busy = 0;
			break;
		}
	}
	uint16_t len = strlen(str);
	if(len >= UART_BUFFER_SIZE)
	{
		len = UART_BUFFER_SIZE -1;
	}

	memcpy(port->tx_buff,str,len);
	port->tx_buff[len] = '\0';
	port->tx_busy = 1;

	HAL_UART_Transmit_DMA(port->huart, port->tx_buff, len);
}

void Uart_TrimRxBuffer(volatile UartComm_t *port)
{
	for (uint16_t i = 0; i < port->rx_len; i++) {
		if (port->rx_buff[i] == '\r' || port->rx_buff[i] == '\n') {
		    port->rx_buff[i] = '\0';
		    break;
		}
	}
}

void Uart_ResetRxBuffer(volatile UartComm_t *port)
{
	memset(port->rx_buff, 0, port->rx_len);
	port->rx_len = 0;
}


