#ifndef CUSTOM_PRINTF_H
#define CUSTOM_PRINTF_H

#include <stdio.h>
#include <stdarg.h>  // For va_list
#include <string.h>  // For strlen()
#include "stm32f4xx_hal.h"  // Adjust for your MCU


// Function prototype
void custom_printf(const char *format, ...);

#endif