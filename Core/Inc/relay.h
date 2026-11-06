/*
 * Relay.h
 *
 *  Created on: 2 Oca 2026
 *      Author: busn
 */

#ifndef INC_RELAY_H_
#define INC_RELAY_H_

#include "main.h"
#include "stdbool.h"
#include "UartComm.h"
#define RELAY_COUNT 4

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} RelayPin;


void Relay_Init(void);
void Relay_Set(int index, uint8_t state);
void Relay_ProcessCommand(char* input);

#endif /* INC_RELAY_H_ */
