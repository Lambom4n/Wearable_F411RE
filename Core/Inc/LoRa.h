#ifndef LORA_H
#define LORA_H

#include "stm32f4xx_hal.h"
#include "RadioLib.h"
#include "SX1261.h"
#include "stm32RadioHal.h"
#include <math.h>
#include "quaternion.h"
#include "compute_direction.h"
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
void LoRa_Task_send(void *pvParameters);
void LoRa_Task_receive(void *pvParameters);
void LoRa_Calibrate_RSSI_tx(void* pvParameters);
void LoRa_Calibrate_RSSI_rx(void* pvParameters);
float LoRa_Get_Distance(float rssi, float snr);


#endif
