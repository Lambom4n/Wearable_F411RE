#ifndef QUATERNION_EXAMPLE_H
#define QUATERNION_EXAMPLE_H
#include "quaternion.h"
extern "C" {
#include "custom_printf.h"
#include "FreeRTOS.h"
#include "task.h"
}
// Function to start the quaternion example task
void QuaternionExample_Task(void* param);

#endif 