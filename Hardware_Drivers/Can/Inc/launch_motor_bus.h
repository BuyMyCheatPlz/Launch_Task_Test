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

typedef struct {
    uint32_t rx_count;
    uint32_t last_rx_ms;
    uint32_t error_code;
    uint32_t esr;
    uint16_t last_std_id;
} LaunchCanDiag_t;

void LaunchMotorBus_Init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2);
void LaunchMotorBus_SetCommand(uint8_t can_index, uint8_t motor_id, int16_t current);
void LaunchMotorBus_Service(uint32_t now_ms);
const LaunchMotorFeedback_t *LaunchMotorBus_Feedback(uint8_t can_index, uint8_t motor_id);
const LaunchCanDiag_t *LaunchMotorBus_Diag(uint8_t can_index);

#endif
