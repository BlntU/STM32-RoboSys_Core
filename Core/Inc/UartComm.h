/*
 * Uart_frame.h
 *
 *  Created on: 2 Oca 2026
 *      Author: busn
 */

#ifndef INC_UARTCOMM_H_
#define INC_UARTCOMM_H_

#include "main.h"
#include <stdint.h>
#include <string.h>

#define UART_BUFFER_SIZE 100

typedef struct {
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_rx;
	DMA_HandleTypeDef *hdma_tx;
	uint8_t rx_buff[UART_BUFFER_SIZE];
	uint8_t tx_buff[UART_BUFFER_SIZE];
	volatile uint16_t rx_len;
	volatile uint8_t rx_ready;
	volatile uint16_t tx_len;
	volatile uint8_t tx_ready;
	volatile uint8_t tx_busy;
} UartComm_t;


void Uart_Init(volatile UartComm_t *port, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx);
void Uart_HandleIdleInterrupt(volatile UartComm_t *port);
void Uart_SendMsg(volatile UartComm_t *port, const char *str);
void Uart_TrimRxBuffer(volatile UartComm_t *port);
void Uart_ResetRxBuffer(volatile UartComm_t *port);



#endif /* INC_UARTCOMM_H_ */
