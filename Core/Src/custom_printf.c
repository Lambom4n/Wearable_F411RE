#include "custom_printf.h"

// Buffer size for formatting (adjust as needed)
#define UART_PORT huart2
#define PRINTF_BUFFER_SIZE  128
extern UART_HandleTypeDef UART_PORT;

void custom_printf(const char *format, ...) {
    char buffer[PRINTF_BUFFER_SIZE];
    va_list args;
    
    // Format the string
    va_start(args, format);
    vsnprintf(buffer, PRINTF_BUFFER_SIZE, format, args);
    va_end(args);
    
    // Transmit the formatted string via UART
    HAL_UART_Transmit(&UART_PORT, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void custom_scanf(const char *format, ...) {
    char buffer[PRINTF_BUFFER_SIZE];

    // Receive the string via UART
    HAL_StatusTypeDef status = HAL_UART_Receive(&UART_PORT, (uint8_t*)buffer, sizeof(uint8_t), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle error in receiving data
        // You can log an error or take other actions here
        return;
    }

    // Parse the received string
    va_list args;
    va_start(args, format);

    int parsed_items = vsscanf(buffer, format, args);
    if (parsed_items == EOF) {
        // Handle parsing error
        // You can log an error or take other actions here
    }

    va_end(args);
}