// quaternion_example.cpp
//  ******************************************************************************
#include "quaternion_example.h"
// Example task that uses the quaternion filter and prints the data
void QuaternionExample_Task(void *param)
{
    // Initialize the quaternion filter
    quaternion_init();

    // Main loop
    while (1)
    {
        // Update the quaternion filter with new IMU data
        quaternion_update();

        // Print the filtered orientation data
        quaternion_print_data();

        // Wait for a short time before the next update
        // Adjust this delay based on your desired update rate
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay (10Hz update rate)
    }
}
