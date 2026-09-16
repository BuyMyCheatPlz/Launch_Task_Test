#ifndef LAUNCH_MOTOR_BUS_H
#define LAUNCH_MOTOR_BUS_H

#include "main.h"
#include <stdint.h>

typedef struct {
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t current;
    uint32_t last_feedback_ms;
    uint8_t online;
} LaunchMotorFeedback_t;

void LaunchMotorBus_Init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2);
void LaunchMotorBus_SetCommand(uint8_t can_index, uint8_t motor_id, int16_t current);
void LaunchMotorBus_Service(uint32_t now_ms);
const LaunchMotorFeedback_t *LaunchMotorBus_Feedback(uint8_t can_index, uint8_t motor_id);

#endif
