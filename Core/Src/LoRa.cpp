#include "LoRa.h"

extern UART_HandleTypeDef huart2;

// Hardware abstraction layer for STM32
STM32RadioHal hal(0, 1, 0, 1, 1, 0);

// Initialize SX1261 LoRa module with hardware parameters (pins: NSS, DIO1, NRST, BUSY)
SX1261 lora = new Module(&hal, 15, 13, 14, 12);

// Queues for inter-task communication
QueueHandle_t distance_queue = NULL;
QueueHandle_t receive_queue = NULL;

// Configuration constants
#define TX_POWER 14.0f
#define FREQUENCY 433.0f
#define PATH_LOSS_EXPONENT_LOW 1.8f
#define PATH_LOSS_EXPONENT_MEDIUM 2.3f
#define PATH_LOSS_EXPONENT_HIGH 3.0f
#define CALIBRATION_DISTANCE 1.0f
#define CALIBRATION_COUNT 20
#define WINDOW_SIZE 5
#define OUTLIER_THRESHOLD 2.0 // For filtering RSSI outliers

#define EMA_ALPHA 0.5 // Exponential Moving Average smoothing factor

float pl_1m = 90.0f; // Placeholder RSSI at 1m, to be calibrated

// Structures for filtering RSSI values
typedef struct {
    float window[WINDOW_SIZE];
    uint8_t index;
    uint8_t filled;
} SlidingWindowFilter;

typedef struct {
    float alpha;
    float filtered_value;
} EMAFilter;

SlidingWindowFilter swf;
EMAFilter ema;

float PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_MEDIUM;

// Initialize sliding window filter
static void swf_init(SlidingWindowFilter *swf)
{
    swf->index = 0;
    swf->filled = 0;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        swf->window[i] = 0.0;
    }
}

// Sliding Window Filter with outlier replacement
static float swf_filter(SlidingWindowFilter *swf, float sample, float snr)
{
    if (snr >= 0.0) // Acceptable signal
    {
        swf->window[swf->index] = sample;
        swf->index = (swf->index + 1) % WINDOW_SIZE;
        if (swf->filled < WINDOW_SIZE) swf->filled++;
    }

    // Compute mean and standard deviation
    float sum = 0.0, sq_sum = 0.0;
    for (uint8_t i = 0; i < swf->filled; i++) sum += swf->window[i];
    float mean = sum / swf->filled;

    for (uint8_t i = 0; i < swf->filled; i++) sq_sum += powf(swf->window[i] - mean, 2);
    float stddev = sqrtf(sq_sum / swf->filled);

    // Check for outlier in the latest sample
    int new_sample_pos = (swf->index - 1 + WINDOW_SIZE) % WINDOW_SIZE;
    float sorted[WINDOW_SIZE];
    for (uint8_t i = 0; i < swf->filled; i++) sorted[i] = swf->window[i];

    // Sort to compute median
    for (uint8_t i = 0; i < swf->filled - 1; i++)
        for (int j = 0; j < swf->filled - i - 1; j++)
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }

    float median = sorted[swf->filled / 2];

    if (fabs(sample - mean) > OUTLIER_THRESHOLD * stddev && snr >= 2.5)
        swf->window[new_sample_pos] = median; // Replace outlier

    return median;
}

// Initialize EMA filter
static void ema_init(EMAFilter *ema, double initial_value)
{
    ema->alpha = EMA_ALPHA;
    ema->filtered_value = initial_value;
}

// Apply EMA filter to sample
static float ema_filter(EMAFilter *ema, double sample)
{
    ema->filtered_value = ema->alpha * sample + (1 - ema->alpha) * ema->filtered_value;
    return ema->filtered_value;
}

// Initialize LoRa module and prompt user for environment setup
void LoRa_Init(void)
{
    int state = lora.begin(FREQUENCY, 125.0, 11, 5, 0x12, 14, 28, 0, false);
    if (state != RADIOLIB_ERR_NONE)
    {
        custom_printf("Error initializing LoRa module\n");
        return;
    }

    custom_printf("Select path loss exponent:\n1. Low (1.8)\n2. Medium (2.3)\n3. High (3.0)\nEnter choice: ");
    uint8_t level = 0;
    HAL_UART_Receive(&huart2, (uint8_t *)&level, sizeof(uint8_t), HAL_MAX_DELAY);

    if (strcmp((char *)&level, "1") == 0) PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_LOW;
    else if (strcmp((char *)&level, "2") == 0) PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_MEDIUM;
    else if (strcmp((char *)&level, "3") == 0) PATH_LOSS_EXPONENT = PATH_LOSS_EXPONENT_HIGH;
    else custom_printf("Invalid choice. Defaulting to Medium (2.3))\n");

    custom_printf("LoRa module initialized\n");
}

// Send data using LoRa
void LoRa_Send(uint8_t *data, uint8_t length)
{
    lora.standby(RADIOLIB_SX126X_STANDBY_RC, true);
    int state = lora.transmit(data, length);
    lora.sleep(true);
    if (state != RADIOLIB_ERR_NONE) custom_printf("Error transmitting data\n");
}

// Estimate distance from RSSI
float LoRa_Get_Distance(float rssi, float snr)
{
    float pl_d = -rssi;
    float filtered_pl_d = swf_filter(&swf, pl_d, snr);
    float smoothed_pl_d = ema_filter(&ema, filtered_pl_d);
    float exponent = (smoothed_pl_d - pl_1m) / (10.0f * PATH_LOSS_EXPONENT);
    return powf(10.0f, exponent);
}

// Receive data using LoRa and compute distance
uint8_t LoRa_Receive(uint8_t *data, uint8_t length, float *distance, float *rssi)
{
    lora.standby(RADIOLIB_SX126X_STANDBY_RC, true);
    int16_t status = lora.receive(data, length);
    lora.sleep(true);
    if (status != RADIOLIB_ERR_NONE)
    {
        if (status == RADIOLIB_ERR_RX_TIMEOUT) return -1;
        custom_printf("Error receiving data\n");
        return -1;
    }

    *rssi = lora.getRSSI();
    float snr = lora.getSNR();
    custom_printf("SNR: %.2f\n", snr);
    *distance = LoRa_Get_Distance(*rssi, snr);
    custom_printf("RSSI: %.2f\n", *rssi);
    return 0;
}

// Calibration task for receiver: average RSSI at known 1m distance
void LoRa_Calibrate_RSSI_rx(void *pvParameters)
{
    float rssi_sum = 0.0f;
    float rssi = 0.0f;
    float distance = 0.0f;
    uint8_t data[4];
    uint8_t count_receive = 0;
    TickType_t start_time = xTaskGetTickCount();
    TickType_t tickcount = xTaskGetTickCount();

    while (xTaskGetTickCount() - start_time < 20000)
    {
        if (LoRa_Receive(data, sizeof(data), &distance, &rssi) == 0)
        {
            rssi_sum += rssi;
            count_receive++;
        }
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(500));
    }

    rssi = rssi_sum / count_receive;
    pl_1m = -rssi;
    custom_printf("RSSI: %.2f\npl_1m: %.2f\nCalibration done rx\n", rssi, pl_1m);
    swf_init(&swf);
    ema_init(&ema, pl_1m);

    // Start main application tasks
    xTaskCreate(LoRa_Task_receive, "LoRa_Task_receive", 512, NULL, 3, NULL);
    xTaskCreate(quaternion_update_task, "quaternion_update", 512, NULL, 1, NULL);
    xTaskCreate(display_task, "display_task", 512, NULL, 2, NULL);
    vTaskDelete(NULL);
}

// LoRa data reception task
void LoRa_Task_receive(void *pvParameters)
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
        if (LoRa_Receive(data_receive, sizeof(data_receive), &distance, &rssi) == 0)
        {
            memcpy(&yaw_data, data_receive, sizeof(yaw_data));
            xQueueSend(distance_queue, &distance, 0);
            xQueueSend(receive_queue, &yaw_data, 0);
        }
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(1700));
    }
}

// Calibration task for transmitter: send packets for 20 seconds
void LoRa_Calibrate_RSSI_tx(void *pvParameters)
{
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x00};
    uint32_t start_time = xTaskGetTickCount();
    uint32_t tickcount = xTaskGetTickCount();

    while (xTaskGetTickCount() - start_time < 20000)
    {
        LoRa_Send((uint8_t *)data, sizeof(data));
        data[0]++;
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(1500));
    }

    swf_init(&swf);
    ema_init(&ema, pl_1m);
    xTaskCreate(LoRa_Task_send, "LoRa_Task_send", 512, NULL, 3, NULL);
    xTaskCreate(quaternion_update_task, "quaternion_update", 512, NULL, 1, NULL);
    custom_printf("Calibration done tx\n");
    vTaskDelete(NULL);
}

// LoRa data transmission task
void LoRa_Task_send(void *pvParameters)
{
    uint8_t data[4] = {0};
    float yaw_data = 0.0f;
    TickType_t tickcount = xTaskGetTickCount();

    while (1)
    {
        yaw_data = quaternion_get_yaw(); // Get orientation
        memcpy(data, &yaw_data, sizeof(yaw_data));
        custom_printf("Sending data: %.2f\n", yaw_data);
        LoRa_Send((uint8_t *)data, sizeof(data));
        vTaskDelayUntil(&tickcount, pdMS_TO_TICKS(3000));
    }
}
