/*
 * Relay.c
 *
 *  Created on: 2 Oca 2026
 *      Author: busn
 */
#include "relay.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

extern UartComm_t uart3_relay;

RelayPin relayPins[RELAY_COUNT] = {
    {GPIOD, GPIO_PIN_12},
    {GPIOD, GPIO_PIN_13},
    {GPIOD, GPIO_PIN_14},
    {GPIOD, GPIO_PIN_15}
};

void Relay_Init(void)
{
	for (int i = 0; i < RELAY_COUNT; i++) {
	        HAL_GPIO_WritePin(relayPins[i].port, relayPins[i].pin, GPIO_PIN_RESET);
	    }
	Uart_SendMsg(&uart3_relay, "STM32 Role Kontrol Basladi \r\n");
}


void Relay_Set(int index, uint8_t state) {

    if (index >= 0 && index < RELAY_COUNT) {

        GPIO_PinState pinState = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(relayPins[index].port, relayPins[index].pin, pinState);
        char msg[32];
        snprintf(msg, sizeof(msg), "Relay %d %s \r\n",
                index + 1, state ? "ON" : "OFF");
        Uart_SendMsg(&uart3_relay, msg);
    }
}

void Relay_ProcessCommand(char* input) {

    if (strcmp(input, "ALL_ON") == 0) {
        for (int i = 0; i < RELAY_COUNT; i++) Relay_Set(i, 1);
    } else if (strcmp(input, "ALL_OFF") == 0) {
        for (int i = 0; i < RELAY_COUNT; i++) Relay_Set(i, 0);
    } else {
        char* space = strchr(input, ' ');
        if (!space) return;
        *space = '\0';
        char* command = input;
        char* targets = space + 1;

        // Trim baştaki boşlukları
        while (*targets == ' ') targets++;

        // Tokenize güvenli şekilde
        char *saveptr;
        char* token = strtok_r(targets, ",", &saveptr);
        while (token != NULL) {
            int relayIndex = atoi(token) - 1;
            if (relayIndex >= 0 && relayIndex < RELAY_COUNT) {
                Relay_Set(relayIndex, strcmp(command, "ON") == 0);
            }
            token = strtok_r(NULL, ",", &saveptr);
        }
    }
}






