#include "compute_direction.h"

uint8_t compute_direction()
{
    // yaw_a is the yaw angle from the quaternion 
    float yaw_a = quaternion_get_yaw();
    // yaw_b is the yaw angle received from LoRa
    float yaw_b = received_yaw_b;

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
void direction_task(void *pvParameters)
{
    while (1)
    {
        uint8_t direction = compute_direction();
        switch (direction)
        {
        case DIRECTION_FORWARD: 
            break;
        case DIRECTION_FORWARD_RIGHT: 
            break;
        case DIRECTION_RIGHT: 
            break;
        case DIRECTION_BACK_RIGHT: 
            break;
        case DIRECTION_BACK: 
            break;
        case DIRECTION_BACK_LEFT: 
            break;
        case DIRECTION_LEFT:
            break;
        case DIRECTION_FORWARD_LEFT: 
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
