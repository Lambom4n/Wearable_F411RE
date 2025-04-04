#include "LoRa.h"
#include "FreeRTOS.h"
#include "task.h"

STM32RadioHal hal(0, 1, 0, 1, 1, 0);
SX1261 lora = new Module(&hal, 15, 13,14, 12);


void LoRa_Init(void)
{
    // Initialize LoRa module
int state = lora.begin(
    433.0,   // Frequency (MHz)
    125.0,   // Bandwidth (kHz)
    7,       // Spreading Factor
    5,       // Coding Rate (4/5)
    0x12,    // Sync Word
    14,      // Output Power (dBm)
    12,       // Preamble Length
    0,      // TCXO Voltage (0 if not used, e.g., 2.0 for 2V TCXO)
    false    // Use LDO Regulator (false = use DC-DC)
);
    if (state != RADIOLIB_ERR_NONE) {
        // There was a problem initializing the module
        // You may want to handle this error condition
        return;
    }
}

void LoRa_Send(uint8_t *data, uint8_t length)
{
    // Send data through LoRa module
    int state = lora.transmit(data, length);
    if (state != RADIOLIB_ERR_NONE) {
        // There was a problem transmitting the data
        // You may want to handle this error condition
        custom_printf("Error transmitting data\n");
        return;
    }
}

void LoRa_Receive(uint8_t *data, uint8_t length)
{
    // Receive data through LoRa module
    int16_t status = lora.receive(data, length);
    if (status != RADIOLIB_ERR_NONE) {
        // There was a problem receiving the data
        // You may want to handle this error condition
        custom_printf("Error receiving data\n");
        return;
    }
    custom_printf("Received data:");
    for (uint8_t i = 0; i < length; i++) {
        custom_printf("%02X ", data[i]);
    }
    custom_printf("\n");
}

void LoRa_Task_send(void *pvParameters)  
{
    // Task for LoRa communication
    uint8_t data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    while (1)
    {
        LoRa_Send(data, sizeof(data));
        custom_printf("Sent data:");
        for (uint8_t i = 0; i < sizeof(data); i++) {
            custom_printf("%02X ", data[i]);
        }
        custom_printf("\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void LoRa_Task_receive(void *pvParameters)
{
    uint8_t data[10];
    while (1)
    {
        LoRa_Receive(data, sizeof(data));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

