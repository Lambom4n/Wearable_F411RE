#include "LoRa.h"

#define SF_2_to_1 10

extern SemaphoreHandle_t Transmit_receive_mutex;
extern UART_HandleTypeDef huart2;

STM32RadioHal hal(0, 1, 0, 1, 1, 0);
SX1261 lora = new Module(&hal, 15, 13, 14, 12);

QueueHandle_t distance_queue = NULL;
QueueHandle_t receive_queue = NULL;

#define TX_POWER 14.0f
#define FREQUENCY 433.0f
#define PATH_LOSS_EXPONENT_LOW 1.5f
#define PATH_LOSS_EXPONENT_MEDIUM 1.8f
#define PATH_LOSS_EXPONENT_HIGH 3.0f
#define CALIBRATION_DISTANCE 1.0f
#define CALIBRATION_COUNT 20
#define WINDOW_SIZE 5
#define OUTLIER_THRESHOLD 2.0 // Mean ± 2 sigma

#define EMA_ALPHA 0.5 // Adjust for smoothing (0 < alpha < 1)

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
    if (snr >= 2.5)
    {
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
        FREQUENCY, // Frequency (MHz)
        125.0,     // Bandwidth (kHz)
        9,        // Spreading Factor
        5,         // Coding Rate (4/7)
        0x12,      // Sync Word
        14,        // Output Power (dBm)
        16,        // Preamble Length
        0,         // TCXO Voltage (0 if not used, e.g., 2.0 for 2V TCXO)
        false      // Use LDO Regulator (false = use DC-DC)
    );
    if (state != RADIOLIB_ERR_NONE)
    {
        // There was a problem initializing the module
        // You may want to handle this error condition
        custom_printf("Error initializing LoRa module\n");
        return;
    }
    // Set path loss exponent based on environment
    custom_printf("Select path loss exponent:\n");
    custom_printf("1. Low (1.5)\n");
    custom_printf("2. Medium (1.8)\n");
    custom_printf("3. High (3.0)\n");
    custom_printf("Enter choice: ");
    uint8_t level = 0;
    HAL_UART_Receive(&huart2, (uint8_t *)&level, sizeof(uint8_t), HAL_MAX_DELAY);
    custom_printf("\n");

    if (strcmp((char *)&level, "1") == 0)
    {
        PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_LOW;
        custom_printf("Path loss exponent set to Low (1.5)\n");
    }
    else if (strcmp((char *)&level, "2") == 0)
    {
        PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_MEDIUM;
        custom_printf("Path loss exponent set to Medium (1.8)\n");
    }
    else if (strcmp((char *)&level, "3") == 0)
    {
        PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_HIGH;
        custom_printf("Path loss exponent set to High (3.0)\n");
    }
    else
    {
        custom_printf("Invalid choice. Defaulting to Medium (1.8))\n");
    }
    custom_printf("LoRa module initialized\n");
}

void LoRa_Send(uint8_t *data, uint8_t length, uint8_t SF)
{   
    // Send data through LoRa module
    xSemaphoreTake(Transmit_receive_mutex, portMAX_DELAY);
    lora.setSpreadingFactor(SF); // Set the spreading factor
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

uint8_t LoRa_Receive(uint8_t *data, uint8_t length, uint8_t SF, float *distance, float *rssi)
{
    // Receive data through LoRa module
    xSemaphoreTake(Transmit_receive_mutex, portMAX_DELAY);
    lora.setSpreadingFactor(SF); // Set the spreading factor
    int16_t status = lora.receive(data, length);
    if (status != RADIOLIB_ERR_NONE)
    {
        // There was a problem receiving the data
        // You may want to handle this error condition
        if (status == RADIOLIB_ERR_RX_TIMEOUT)
        {
            xSemaphoreGive(Transmit_receive_mutex);
            custom_printf("Timeout receiving data\n");
            return -1;
        }
        else
        {
            xSemaphoreGive(Transmit_receive_mutex);
            custom_printf("Error receiving data\n");
            return -1;
        }
    }
    xSemaphoreGive(Transmit_receive_mutex);
    *rssi = lora.getRSSI();
    float snr = lora.getSNR();
    custom_printf("data received: ");
    for (uint8_t i = 0; i < length; i++)
    {
        custom_printf("%d ", data[i]);
    }
    custom_printf("SNR: %.2f\n", snr);
    // custom_printf("Received data:\n");
    // for (uint8_t i = 0; i < length; i++) {
    //     custom_printf("%d ", data[i]);
    // }
    // custom_printf("\n");
    *distance = LoRa_Get_Distance(*rssi, snr);
    custom_printf("RSSI: %.2f\n", *rssi);
    custom_printf("Distance: %.2f\n\n", *distance);
    return 0;
}


void LoRa_Calibrate_RSSI_rx_1(void *pvParameters)
{
    float rssi_sum = 0.0f;
    float rssi = 0.0f;
    float distance = 0.0f;
    uint8_t data[5];
    uint8_t count_receive = 0;
    TickType_t start_time = xTaskGetTickCount();
    TickType_t tickcount = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_time < 20000)
    {
        if (LoRa_Receive(data, sizeof(data), SF_2_to_1, &distance, &rssi) == 0)
        {
            rssi_sum += rssi;
            count_receive++;
        }
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(1000)); // Receive every 1 second
    }
    rssi = rssi_sum / count_receive;
    pl_1m = -rssi;
    custom_printf("RSSI: %.2f\n", rssi);
    custom_printf("pl_1m: %.2f\n", pl_1m);
    custom_printf("Calibration done rx\n");
    custom_printf("----------------------\n");
    custom_printf("create main application\n");
    custom_printf("----------------------\n");
    swf_init(&swf);
    ema_init(&ema, pl_1m); // Initialize EMA filter with initial value pl_1m
    BaseType_t status = xTaskCreate(LoRa_Task_receive_1, "LoRa_Task_receive", 512, NULL, 3, NULL);
    vTaskDelete(NULL);
}


void LoRa_Task_receive_1(void *pvParameters)
{
    uint8_t data_receive[4] = {0};
    float distance = 0.0f;
    float yaw_data = 0.0f;
    float rssi = 0.0f;
    distance_queue = xQueueCreate(8, sizeof(float));
    receive_queue = xQueueCreate(8, sizeof(float));
    TickType_t tickcount = xTaskGetTickCount();
    while (1)
    {
        if (LoRa_Receive(data_receive, sizeof(data_receive), SF_2_to_1, &distance, &rssi) == 0)
        {
            memcpy(&yaw_data, data_receive, sizeof(yaw_data));
            xQueueSend(distance_queue, &distance, 0);
            xQueueSend(receive_queue, &yaw_data, 0);
        }
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(1000)); // Receive every 1 second
    }
}


void LoRa_Calibrate_RSSI_tx_2(void *pvParameters)
{   
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x00};
    uint32_t start_time = xTaskGetTickCount();
    uint32_t tickcount = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_time < 20000)
    {   
        LoRa_Send((uint8_t *)data, sizeof(data), SF_2_to_1);
        data[0]++;
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(3000)); // Send every 1 second
    }
    swf_init(&swf);
    ema_init(&ema, pl_1m); // Initialize EMA filter with initial value pl_1m
    BaseType_t status = xTaskCreate(LoRa_Task_send_2, "LoRa_Task_send", 512, NULL, 3, NULL);
    custom_printf("Calibration done tx\n");
    vTaskDelete(NULL);
}


void LoRa_Task_send_2(void *pvParameters)
{
    // Task for LoRa communication
    uint8_t example_data[4] = {0x01, 0x02, 0x03, 0x00};
    uint8_t data[4] = {0};
    float yaw_data = 0.0f;
    TickType_t tickcount = xTaskGetTickCount();
    while (1)
    {   
        yaw_data = quaternion_get_yaw();
        memcpy(data, &yaw_data, sizeof(yaw_data));
        custom_printf("Sending data: %.2f\n", yaw_data);
        LoRa_Send((uint8_t *)example_data, sizeof(example_data), SF_2_to_1);
        example_data[0]++;
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(3000)); // Send every 1 second
    }
}

