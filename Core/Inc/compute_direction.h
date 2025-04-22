#ifndef COMPUTE_DIRECTION_H
#define COMPUTE_DIRECTION_H

#include "quaternion.h"
#include "LoRa.h"

#define DIRECTION_FORWARD        0
#define DIRECTION_FORWARD_RIGHT  1
#define DIRECTION_RIGHT          2
#define DIRECTION_BACK_RIGHT     3
#define DIRECTION_BACK           4
#define DIRECTION_BACK_LEFT      5
#define DIRECTION_LEFT           6
#define DIRECTION_FORWARD_LEFT   7

typedef struct
{
    uint8_t direction;
    float distance;
} DirectionData;


uint8_t compute_direction();
void display_task(void *pvParameters);

#endif
