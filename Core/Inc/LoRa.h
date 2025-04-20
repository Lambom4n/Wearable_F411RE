#ifndef LORA_H
#define LORA_H

#include "stm32f4xx_hal.h"
#include "RadioLib.h"
#include "SX1261.h"
#include "stm32RadioHal.h"
extern "C" {
#include "FreeRTOS.h"
#include "custom_printf.h"
}

void LoRa_Init(void);
void LoRa_Send(uint8_t *data, uint8_t length);
void LoRa_Receive(uint8_t *data, uint8_t length);
void LoRa_Task_send(void *pvParameters);
void LoRa_Task_receive(void *pvParameters);
extern float received_yaw_b; // Global variable to store received yaw value
#endif
