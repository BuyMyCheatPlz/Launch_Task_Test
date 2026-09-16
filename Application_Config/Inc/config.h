#ifndef LAUNCH_CONFIG_H
#define LAUNCH_CONFIG_H

/* 硬件映射：接线或电机 ID 改变时只修改本节。 */
#define LAUNCH_LEFT_FLYWHEEL_CAN       1U       /* 1=CAN1，2=CAN2 */
#define LAUNCH_LEFT_FLYWHEEL_ID        2U        /* DJI 电机 ID：1..8 */
#define LAUNCH_RIGHT_FLYWHEEL_CAN      1U
#define LAUNCH_RIGHT_FLYWHEEL_ID       3U
#define LAUNCH_FEEDER_CAN              2U
#define LAUNCH_FEEDER_ID               5U
/* 电机方向标定：期望正转填 +1.0f；方向相反时改为 -1.0f。 */
#define LAUNCH_LEFT_FLYWHEEL_DIR       1.0f
#define LAUNCH_RIGHT_FLYWHEEL_DIR     -1.0f
#define LAUNCH_FEEDER_DIR              1.0f

/* 发射机构参数与安全默认值。M2006 编码器位于电机轴。 */
#define LAUNCH_FEEDER_GEAR_RATIO       36.0f
/* 每发弹丸对应的拨盘输出轴转角（默认 40°）。更换弹仓、拨盘槽数或机构传动后，
 * 只修改此宏；单发、锁相环、完成计数和卡弹退弹都会自动使用新角度。 */
#define LAUNCH_FEEDER_STEP_DEG         40.0f
#define LAUNCH_DEFAULT_SPEED_RPM       6000.0f
#define LAUNCH_DEFAULT_BULLET_COUNT    1U
#define LAUNCH_DEFAULT_SINGLE_INTERVAL_MS 50U
/* 1：使用原工程锁相环连发规划；0：逐发规划，间隔可由 VOFA 在线调整。 */
#define LAUNCH_ENABLE_CONTINUOUS_PLL   1U
/* 卡弹保护：拨盘目标误差大且反馈速度持续接近零时判卡弹；拨盘反退五发后，
 * 全部电机停机，直至下一次 START。 */
#define LAUNCH_ENABLE_JAM_PROTECTION   0U
#define LAUNCH_JAM_STALL_MS            300U
#define LAUNCH_JAM_MIN_ERROR_DEG       20.0f
#define LAUNCH_JAM_ZERO_SPEED_RPM      30.0f
#define LAUNCH_JAM_REVERSE_BULLETS     5U
#define LAUNCH_FLYWHEEL_SPINUP_MS      500U
#define LAUNCH_CONTROL_PERIOD_MS       1U
#define LAUNCH_VOFA_PERIOD_MS          20U
#define LAUNCH_MOTOR_OFFLINE_MS        100U

/* 从原云台工程迁移的闭环参数。 */
#define LAUNCH_M3508_KP                18.0f
#define LAUNCH_M3508_KI                20.0f
#define LAUNCH_M3508_KD                100.0f
#define LAUNCH_M3508_FILTER_ALPHA      0.50f
#define LAUNCH_M3508_CURRENT_LIMIT     16384.0f
#define LAUNCH_M2006_KP                12.0f
#define LAUNCH_M2006_KI                1.2f
#define LAUNCH_M2006_FILTER_ALPHA      0.70f
#define LAUNCH_M2006_CURRENT_LIMIT     6500.0f
#define LAUNCH_FEEDER_ANGLE_KP         300.0f   /* 拨盘输出角度 → 电机 rpm */
#define LAUNCH_FEEDER_MAX_RPM          4200.0f
#define LAUNCH_FEEDER_DEADBAND_DEG     0.8f
#define LAUNCH_FEEDER_PLL_KP_RPM_PER_DEG 60.0f
#define LAUNCH_FEEDER_PLL_MAX_RPM      5600.0f

#endif
