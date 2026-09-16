#include "launch_fire_planner.h"
#include "config.h"

static float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void LaunchFirePlanner_Reset(LaunchFirePlanner_t *planner, float actual_deg,
                             uint32_t now_ms)
{
    if (planner == 0) return;
    planner->target_deg = actual_deg;
    planner->start_deg = actual_deg;
    planner->issued_count = 0U;
    planner->last_update_ms = now_ms;
    planner->initialized = 1U;
}

void LaunchFirePlanner_Update(LaunchFirePlanner_t *planner, float actual_deg,
                              uint32_t now_ms, uint32_t interval_ms,
                              uint32_t requested_count, uint8_t enabled)
{
    float step = LAUNCH_FEEDER_DIR * LAUNCH_FEEDER_STEP_DEG;
    if (planner == 0) return;
    if (interval_ms == 0U) interval_ms = 1U;
    if ((planner->initialized == 0U) || (enabled == 0U))
    {
        LaunchFirePlanner_Reset(planner, actual_deg, now_ms);
        return;
    }
#if (LAUNCH_ENABLE_CONTINUOUS_PLL != 0U)
    uint32_t elapsed_ms;
    /* 原工程行为：相位参考按“每个间隔一发”连续推进。卡弹导致相位误差超过
     * 两发时重新锁相，避免恢复时追赶产生危险的加速度。 */
    elapsed_ms = now_ms - planner->last_update_ms;
    planner->last_update_ms = now_ms;
    if (planner->issued_count < requested_count)
    {
        planner->target_deg += step * (float)elapsed_ms / (float)interval_ms;
        if ((step > 0.0f) && (planner->target_deg > planner->start_deg + (float)requested_count * step))
            planner->target_deg = planner->start_deg + (float)requested_count * step;
        if ((step < 0.0f) && (planner->target_deg < planner->start_deg + (float)requested_count * step))
            planner->target_deg = planner->start_deg + (float)requested_count * step;
        if (((planner->target_deg - actual_deg) > (2.0f * LAUNCH_FEEDER_STEP_DEG)) ||
            ((planner->target_deg - actual_deg) < (-2.0f * LAUNCH_FEEDER_STEP_DEG)))
            planner->target_deg = actual_deg;
        {
            float progress = (planner->target_deg - planner->start_deg) / step;
            if (progress < 0.0f) progress = 0.0f;
            if (progress > (float)requested_count) progress = (float)requested_count;
            planner->issued_count = (uint32_t)progress;
        }
    }
#else
    /* 单发模式：每个间隔精确下发一个 40° 拨盘步进；VOFA 可直接修改间隔。 */
    if ((planner->issued_count < requested_count) &&
        ((now_ms - planner->last_update_ms) >= interval_ms))
    {
        planner->target_deg += step;
        ++planner->issued_count;
        planner->last_update_ms = now_ms;
    }
#endif
}

float LaunchFirePlanner_TargetSpeedRpm(const LaunchFirePlanner_t *planner,
                                       float actual_deg, uint32_t interval_ms)
{
    float error;
    if (planner == 0) return 0.0f;
    error = planner->target_deg - actual_deg;
#if (LAUNCH_ENABLE_CONTINUOUS_PLL != 0U)
    if (interval_ms == 0U) interval_ms = 1U;
    return clamp(LAUNCH_FEEDER_DIR * LAUNCH_FEEDER_STEP_DEG *
                 LAUNCH_FEEDER_GEAR_RATIO * 60000.0f /
                 (360.0f * (float)interval_ms) +
                 LAUNCH_FEEDER_PLL_KP_RPM_PER_DEG * error,
                 LAUNCH_FEEDER_PLL_MAX_RPM);
#else
    if ((error < LAUNCH_FEEDER_DEADBAND_DEG) &&
        (error > -LAUNCH_FEEDER_DEADBAND_DEG)) return 0.0f;
    return clamp(error * LAUNCH_FEEDER_ANGLE_KP, LAUNCH_FEEDER_MAX_RPM);
#endif
}
