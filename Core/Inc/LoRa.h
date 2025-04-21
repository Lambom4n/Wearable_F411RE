#ifndef LORA_H
#define LORA_H

#include "stm32f4xx_hal.h"
#include "RadioLib.h"
#include "SX1261.h"
#include "stm32RadioHal.h"
#include <math.h>
extern "C" {
#include "FreeRTOS.h"
#include "custom_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
}


void LoRa_Init(void);
void LoRa_Send(uint8_t *data, uint8_t length);
void LoRa_Receive(uint8_t *data, uint8_t length);
void LoRa_Task_send_1(void *pvParameters);
void LoRa_Task_receive_1(void *pvParameters);
void LoRa_Calibrate_RSSI_tx_1(void* pvParameters);
void LoRa_Calibrate_RSSI_rx_1(void* pvParameters);
void LoRa_Task_send_2(void *pvParameters);
void LoRa_Task_receive_2(void *pvParameters);
void LoRa_Calibrate_RSSI_tx_2(void* pvParameters);
void LoRa_Calibrate_RSSI_rx_2(void* pvParameters);
float LoRa_Get_Distance(float rssi, float snr);


#endif
