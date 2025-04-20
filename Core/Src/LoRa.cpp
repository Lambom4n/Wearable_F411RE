#include "LoRa.h"
#include "FreeRTOS.h"
#include "task.h"
#include "quaternion.h"

STM32RadioHal hal(0, 1, 0, 1, 1, 0);
SX1261 lora = new Module(&hal, 15, 13, 14, 12);
float received_yaw_b = 0.0f;
void LoRa_Init(void)
{
    // Initialize LoRa module
    int state = lora.begin(
        433.0, // Frequency (MHz)
        125.0, // Bandwidth (kHz)
        12,    // Spreading Factor
        7,     // Coding Rate (4/5)
        0x12,  // Sync Word
        14,    // Output Power (dBm)
        12,    // Preamble Length
        0,     // TCXO Voltage (0 if not used, e.g., 2.0 for 2V TCXO)
        false  // Use LDO Regulator (false = use DC-DC)
    );
    if (state != RADIOLIB_ERR_NONE)
    {
        // There was a problem initializing the module
        // You may want to handle this error condition
        return;
    }
}

void LoRa_Send(uint8_t *data, uint8_t length)
{
    // Send data through LoRa module
    int state = lora.transmit(data, length);
    if (state != RADIOLIB_ERR_NONE)
    {
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
    if (status != RADIOLIB_ERR_NONE)
    {
        // There was a problem receiving the data
        // You may want to handle this error condition
        custom_printf("Error receiving data\n");
        return;
    }
    custom_printf("Received data:");
    for (uint8_t i = 0; i < length; i++)
    {
        custom_printf("%02X ", data[i]);
    }
    custom_printf("\n");
}

void LoRa_Task_send(void *pvParameters)
{
    // Use a byte array to ensure safe transmission
    uint8_t data[4]; // 4 bytes for float yaw

    while (1)
    {
        // Read yaw value from quaternion
        float yaw = quaternion_get_yaw();

        // Pack float into bytes (handle endianness)
        union
        {
            float f;
            uint8_t bytes[4];
        } float_union;
        float_union.f = yaw;

        // Reverse byte order for big-endian transmission
        data[0] = float_union.bytes[3];
        data[1] = float_union.bytes[2];
        data[2] = float_union.bytes[1];
        data[3] = float_union.bytes[0];

        LoRa_Send(data, sizeof(data));
        custom_printf("Sent yaw: %.2f°\n", yaw);

        // // Assign value in a portable way (big-endian)
        // uint16_t value = 0x0102;       // Example value
        // data[0] = (value >> 8) & 0xFF; // High byte (big-endian)
        // data[1] = value & 0xFF;        // Low byte

        // // Send data
        // LoRa_Send(data, sizeof(data));

        // // Print sent value (as integer and hex)
        // custom_printf("Sent data: %d (0x%04X)\n", value, value);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void LoRa_Task_receive(void *pvParameters)
{
    uint8_t data[10];
    while (1)
    {
        LoRa_Receive(data, sizeof(data));
        union
        {
            float f;
            uint8_t bytes[4];
        } float_union;

        // Reverse byte order for little-endian MCU
        float_union.bytes[3] = data[0];
        float_union.bytes[2] = data[1];
        float_union.bytes[1] = data[2];
        float_union.bytes[0] = data[3];

        received_yaw_b = float_union.f;
        custom_printf("Received yaw: %.2f°\n", received_yaw_b);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
