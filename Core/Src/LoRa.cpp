#include "LoRa.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern SemaphoreHandle_t Transmit_receive_mutex;
extern UART_HandleTypeDef huart2;

STM32RadioHal hal(0, 1, 0, 1, 1, 0);
SX1261 lora = new Module(&hal, 15, 13, 14, 12);

#define TX_POWER 14.0f
#define FREQUENCY 433.0f
#define PATH_LOSS_EXPONENT_LOW 1.9f
#define PATH_LOSS_EXPONENT_MEDIUM 2.2f
#define PATH_LOSS_EXPONENT_HIGH 3.0f
#define CALIBRATION_DISTANCE 1.0f
#define CALIBRATION_COUNT 20
#define WINDOW_SIZE 5
#define OUTLIER_THRESHOLD 2.0 // Mean ± 2 sigma

#define EMA_ALPHA 0.4 // Adjust for smoothing (0 < alpha < 1)

float pl_1m = 90.0f;

typedef struct
{
    float window[WINDOW_SIZE];
    uint8_t index;  // Current position in the circular buffer
    uint8_t filled; // Number of valid samples in the window
} SlidingWindowFilter;

// EMA Filter Struct
typedef struct
{
    float alpha;
    float filtered_value;
} EMAFilter;

SlidingWindowFilter swf;
EMAFilter ema;

float PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_MEDIUM; // Default value

static void swf_init(SlidingWindowFilter *swf)
{
    swf->index = 0;
    swf->filled = 0;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        swf->window[i] = 0.0;
    }
}

static float swf_filter(SlidingWindowFilter *swf, float sample, float snr)
{
    // Add new sample to circular buffer
    if(snr >=2.5) {
        swf->window[swf->index] = sample;
        swf->index = (swf->index + 1) % WINDOW_SIZE;
        if (swf->filled < WINDOW_SIZE)
            swf->filled++;
    }

    // Compute mean and standard deviation of the window
    float sum = 0.0, sq_sum = 0.0;
    for (uint8_t i = 0; i < swf->filled; i++)
    {
        sum += swf->window[i];
    }
    float mean = sum / swf->filled;

    for (uint8_t i = 0; i < swf->filled; i++)
    {
        sq_sum += (swf->window[i] - mean) * (swf->window[i] - mean);
    }
    float stddev = sqrt(sq_sum / swf->filled);

    // Check if the new sample is an outlier
    int new_sample_pos = (swf->index - 1 + WINDOW_SIZE) % WINDOW_SIZE;
    // Replace outlier with the median of the window
    float sorted[WINDOW_SIZE];
    for (uint8_t i = 0; i < swf->filled; i++)
    {
        sorted[i] = swf->window[i];
    }
    // Bubble sort (simple for small window sizes)
    for (uint8_t i = 0; i < swf->filled - 1; i++)
    {
        for (int j = 0; j < swf->filled - i - 1; j++)
        {
            if (sorted[j] > sorted[j + 1])
            {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    float median = sorted[swf->filled / 2];
    if (fabs(sample - mean) > OUTLIER_THRESHOLD * stddev && snr >= 2.5)
    {
        // Replace outlier with median
        swf->window[new_sample_pos] = median; // Update window with median
    }
    return median;
}

static void ema_init(EMAFilter *ema, double initial_value)
{
    ema->alpha = EMA_ALPHA;
    ema->filtered_value = initial_value;
}

// EMA Filter
static float ema_filter(EMAFilter *ema, double sample)
{
    ema->filtered_value = ema->alpha * sample + (1 - ema->alpha) * ema->filtered_value;
    return ema->filtered_value;
}

void LoRa_Init(void)
{
    // Initialize LoRa module
    int state = lora.begin(
        433.0, // Frequency (MHz)
        125.0, // Bandwidth (kHz)
        12,    // Spreading Factor
        7,     // Coding Rate (4/7)
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
    // Set path loss exponent based on environment
    custom_printf("Select path loss exponent:\n");
    custom_printf("1. Low (1.4)\n");
    custom_printf("2. Medium (2.2)\n");
    custom_printf("3. High (3.0)\n");
    custom_printf("Enter choice: ");
    uint8_t level = 0;
    HAL_UART_Receive(&huart2, (uint8_t *)&level, sizeof(uint8_t), HAL_MAX_DELAY);
    custom_printf("\n");

    if (strcmp((char *)&level, "1") == 0)
    {
        PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_LOW;
        custom_printf("Path loss exponent set to Low (1.3)\n");
    }
    else if (strcmp((char *)&level, "2") == 0)
    {
        PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_MEDIUM;
        custom_printf("Path loss exponent set to Medium (2.2)\n");
    }
    else if (strcmp((char *)&level, "3") == 0)
    {
        PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_HIGH;
        custom_printf("Path loss exponent set to High (3.0)\n");
    }
    else
    {
        custom_printf("Invalid choice. Defaulting to Medium (2.2)\n");
    }
    custom_printf("LoRa module initialized\n");
}

void LoRa_Send(uint8_t *data, uint8_t length)
{
    // Send data through LoRa module
    xSemaphoreTake(Transmit_receive_mutex, portMAX_DELAY);
    int state = lora.transmit(data, length);
    xSemaphoreGive(Transmit_receive_mutex);
    if (state != RADIOLIB_ERR_NONE)
    {
        // There was a problem transmitting the data
        // You may want to handle this error condition
        custom_printf("Error transmitting data\n");
        return;
    }
}

float LoRa_Get_Distance(float rssi, float snr)
{
    // Calculate 20 * log10(frequency)
    float pl_d = -rssi;
    float filtered_pl_d = swf_filter(&swf, pl_d, snr);
    float smoothed_pl_d = ema_filter(&ema, filtered_pl_d);
    custom_printf("Filtered RSSI: %.2f\n", smoothed_pl_d);
    // Apply the formula
    float exponent = (smoothed_pl_d - pl_1m) / (10.0f * PATH_LOSS_EXPONENT);
    float distance = powf(10.0f, exponent);
    return distance;
}

void LoRa_Receive(uint8_t *data, uint8_t length, float *rssi)
{
    // Receive data through LoRa module
    xSemaphoreTake(Transmit_receive_mutex, portMAX_DELAY);
    int16_t status = lora.receive(data, length);
    if (status != RADIOLIB_ERR_NONE)
    {
        // There was a problem receiving the data
        // You may want to handle this error condition
        if (status == RADIOLIB_ERR_RX_TIMEOUT)
        {
            xSemaphoreGive(Transmit_receive_mutex);
            custom_printf("Timeout receiving data\n");
            return;
            // uint32_t start_time = xTaskGetTickCount();
            // while(status != RADIOLIB_ERR_NONE && xTaskGetTickCount() - start_time < 1000) {
            //     status = lora.receive(data, length);
            //     vTaskDelay(pdMS_TO_TICKS(100));
            // }
            // if (status != RADIOLIB_ERR_NONE) {

            //     xSemaphoreGive(Transmit_receive_mutex);
            //     custom_printf("Timeout receiving data\n");
            //     return;
            // }
        }
        else
        {
            xSemaphoreGive(Transmit_receive_mutex);
            custom_printf("Error receiving data\n");
            return;
        }
    }
    xSemaphoreGive(Transmit_receive_mutex);
    *rssi = lora.getRSSI();
    float snr = lora.getSNR();
    custom_printf("SNR: %.2f\n", snr);
    // custom_printf("Received data:\n");
    // for (uint8_t i = 0; i < length; i++) {
    //     custom_printf("%d ", data[i]);
    // }
    // custom_printf("\n");
    uint16_t data16 = (data[0] << 8) | data[1];
    float distance = LoRa_Get_Distance(*rssi, snr);
    custom_printf("RSSI: %.2f\n", *rssi);
    custom_printf("Distance: %.2f\n\n", distance);
}

void LoRa_Calibrate_RSSI_tx(void *pvParameters)
{
    uint8_t data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint32_t start_time = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_time < 15000)
    {
        LoRa_Send((uint8_t *)data, sizeof(data));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    custom_printf("Calibration done\n");
    vTaskDelete(NULL);
}

void LoRa_Calibrate_RSSI_rx(void *pvParameters)
{
    float rssi_sum = 0.0f;
    float rssi = 0.0f;
    uint8_t data[5];
    uint8_t count_receive = 0;
    TickType_t start_time = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_time < 15000)
    {
        LoRa_Receive(data, sizeof(data), &rssi);
        rssi_sum += rssi;
        count_receive++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    rssi = rssi_sum / count_receive;
    pl_1m = -rssi - 3.0f;
    custom_printf("RSSI: %.2f\n", rssi);
    custom_printf("pl_1m: %.2f\n", pl_1m);
    custom_printf("Calibration done\n");
    custom_printf("----------------------\n");
    custom_printf("create main application\n");
    custom_printf("----------------------\n");
    swf_init(&swf);
    ema_init(&ema, pl_1m); // Initialize EMA filter with initial value 0.0
    BaseType_t status = xTaskCreate(LoRa_Task_receive, "LoRa_Task_receive", 2048, NULL, 1, NULL);
    if (status != pdPASS)
    {
        custom_printf("Failed to create LoRa_Task_receive\n");
    }
    vTaskDelete(NULL);
}

void LoRa_Task_send(void *pvParameters)
{
    // Task for LoRa communication
    uint8_t data[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    while (1)
    {
        LoRa_Send(data, sizeof(data));
        custom_printf("Sent data:");
        for (uint8_t i = 0; i < sizeof(data); i++)
        {
            custom_printf("%02X ", data[i]);
        }
        custom_printf("\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void LoRa_Task_receive(void *pvParameters)
{
    uint16_t data_receive = 0;
    float rssi = 0.0f;
    while (1)
    {
        // LoRa_Send((uint8_t*)&data_send, sizeof(data_send));

        LoRa_Receive((uint8_t *)&data_receive, sizeof(data_receive), &rssi);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
