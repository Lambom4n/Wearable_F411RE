#include "compute_direction.h"

extern QueueHandle_t receive_queue;
extern QueueHandle_t distance_queue; // Distance queue from LoRa

uint8_t compute_direction()
{
    // yaw_a is the yaw angle from the quaternion 
    float yaw_a = quaternion_get_yaw();
    // yaw_b is the yaw angle received from LoRa
    float yaw_b;
    xQueueReceive(receive_queue, &yaw_b, portMAX_DELAY);
    // Calculate angular difference
    float delta_yaw = yaw_b - yaw_a;

    // Normalize to [-180°, 180°]
    delta_yaw = fmodf(delta_yaw + 180.0f, 360.0f) - 180.0f;

    // Determine 8-way direction
    if (delta_yaw > 157.5f || delta_yaw <= -157.5f)
    {
        return DIRECTION_BACK;
    }
    else if (delta_yaw > 112.5f)
    {
        return DIRECTION_BACK_RIGHT;
    }
    else if (delta_yaw > 67.5f)
    {
        return DIRECTION_RIGHT;
    }
    else if (delta_yaw > 22.5f)
    {
        return DIRECTION_FORWARD_RIGHT;
    }
    else if (delta_yaw > -22.5f)
    {
        return DIRECTION_FORWARD;
    }
    else if (delta_yaw > -67.5f)
    {
        return DIRECTION_FORWARD_LEFT;
    }
    else if (delta_yaw > -112.5f)
    {
        return DIRECTION_LEFT;
    }
    else
    {
        return DIRECTION_BACK_LEFT;
    }
}

// Update the display connected to Arduino
void display_task(void *pvParameters)
{
    DirectionData direction_data;
    while (1)
    {
        uint8_t direction = compute_direction();
        float distance = 0.0f;
        xQueueReceive(distance_queue, &distance, 0);
        direction_data.direction = direction;
        direction_data.distance = distance;
        custom_printf("Direction: %d, Distance: %.2f\n", direction_data.direction, direction_data.distance);
        vTaskDelay(pdMS_TO_TICKS(3000)); // Update every 1 second
    }
}
