#ifndef LAUNCH_FIRE_PLANNER_H
#define LAUNCH_FIRE_PLANNER_H

#include <stdint.h>

/* 任务层发射规划器：不依赖 CAN、UART、PID 或 FreeRTOS。 */
typedef struct
{
    float target_deg;
    float start_deg;
    uint32_t issued_count;
    uint32_t last_update_ms;
    uint8_t initialized;
} LaunchFirePlanner_t;

void LaunchFirePlanner_Reset(LaunchFirePlanner_t *planner, float actual_deg,
                             uint32_t now_ms);
void LaunchFirePlanner_Update(LaunchFirePlanner_t *planner, float actual_deg,
                              uint32_t now_ms, uint32_t interval_ms,
                              uint32_t requested_count, uint8_t enabled);
float LaunchFirePlanner_TargetSpeedRpm(const LaunchFirePlanner_t *planner,
                                       float actual_deg, uint32_t interval_ms);

#endif
