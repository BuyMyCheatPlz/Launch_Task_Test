#ifndef LAUNCH_CONFIG_H
#define LAUNCH_CONFIG_H

/* 硬件映射：接线或电机 ID 改变时只修改本节。 */
#define LAUNCH_LEFT_FLYWHEEL_CAN       1U       /* 1=CAN1，2=CAN2 */
#define LAUNCH_LEFT_FLYWHEEL_ID        2U        /* DJI 电机 ID：1..8 */
#define LAUNCH_RIGHT_FLYWHEEL_CAN      1U
#define LAUNCH_RIGHT_FLYWHEEL_ID       3U
#define LAUNCH_FEEDER_CAN              1U
#define LAUNCH_FEEDER_ID               1U
/* 可选 pitch 轴 2006：宏关闭时不编译控制和 VOFA 通道。 */
#define LAUNCH_ENABLE_PITCH_M2006      0U
#define LAUNCH_PITCH_CAN               2U
#define LAUNCH_PITCH_ID                1U
/* CAN2 诊断打印：排查 CAN2 线、终端、电调供电或中断问题时临时打开。
 * 关闭时底层仍保留诊断统计数据，只是不追加到 VOFA 打印通道。 */
#define LAUNCH_ENABLE_CAN2_DIAG        0U
/* 电机方向标定：期望正转填 +1.0f；方向相反时改为 -1.0f。 */
#define LAUNCH_LEFT_FLYWHEEL_DIR       1.0f
#define LAUNCH_RIGHT_FLYWHEEL_DIR     -1.0f
#define LAUNCH_FEEDER_DIR              1.0f
#define LAUNCH_PITCH_DIR               1.0f

/* 发射机构参数与安全默认值。M2006 编码器位于电机轴。 */
#define LAUNCH_FEEDER_GEAR_RATIO       36.0f
/* 每发弹丸对应的拨盘输出轴转角（默认 40°）。更换弹仓、拨盘槽数或机构传动后，
 * 只修改此宏；单发、锁相环、完成计数和卡弹退弹都会自动使用新角度。 */
#define LAUNCH_FEEDER_STEP_DEG         40.0f
#define LAUNCH_DEFAULT_SPEED_RPM       6000.0f
#define LAUNCH_DEFAULT_BULLET_COUNT    1U
#define LAUNCH_DEFAULT_SINGLE_INTERVAL_MS 66U

/* 发射模式选择：
 * 1：锁相环连发模式，按 INTERVAL 作为每发周期连续推进；
 * 0：单发计数模式，每发走 LAUNCH_FEEDER_STEP_DEG，发射间隔可由 VOFA 的 INTERVAL 调整。 */
#define LAUNCH_ENABLE_PLL_FIRE_MODE    0U
/* 兼容旧名字，业务代码仍统一使用这个宏判断模式。 */
#define LAUNCH_ENABLE_CONTINUOUS_PLL   LAUNCH_ENABLE_PLL_FIRE_MODE

/* 卡弹保护：拨盘目标误差大且反馈速度持续接近零时判卡弹；拨盘反退一发后，
 * 全部电机停机，直至下一次 START。 */
#define LAUNCH_ENABLE_JAM_PROTECTION   1U
#define LAUNCH_JAM_STALL_MS            300U
#define LAUNCH_JAM_MIN_ERROR_DEG       20.0f
#define LAUNCH_JAM_ZERO_SPEED_RPM      30.0f
#define LAUNCH_JAM_REVERSE_BULLETS     1U
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
/* 两颗摩擦轮差速补偿环：将左右实测转速按方向统一后比较，差值经比例环以
 * 等量反向修正两侧目标转速。0=关闭，1=开启。 */
#define LAUNCH_ENABLE_FLYWHEEL_DIFF_LOOP 1U
#define LAUNCH_FLYWHEEL_DIFF_KP        0.30f   /* 左右速度差 rpm → 修正 rpm */
#define LAUNCH_FLYWHEEL_DIFF_MAX_CORRECTION_RPM 500.0f
/* 差速环开启时，只有左右归一化转速差连续小于该阈值一段时间后，
 * 才允许拨盘发弹。 */
#define LAUNCH_FLYWHEEL_DIFF_READY_RPM 100.0f
#define LAUNCH_FLYWHEEL_DIFF_READY_MS  100U
#define LAUNCH_M2006_KP                12.0f
#define LAUNCH_M2006_KI                1.2f
#define LAUNCH_M2006_FILTER_ALPHA      0.70f
#define LAUNCH_M2006_CURRENT_LIMIT     6500.0f
#define LAUNCH_FEEDER_ANGLE_KP         300.0f   /* 拨盘输出角度 → 电机 rpm */
#define LAUNCH_FEEDER_MAX_RPM          4200.0f
/* 目标发数下发完成后进入位置保持，使用较低速度上限抑制重负载释放后的过冲。 */
#define LAUNCH_FEEDER_HOLD_MAX_RPM     1200.0f
#define LAUNCH_FEEDER_DEADBAND_DEG     0.8f
#define LAUNCH_FEEDER_PLL_KP_RPM_PER_DEG 60.0f
#define LAUNCH_FEEDER_PLL_MAX_RPM      5600.0f

/* pitch 轴 2006 位置环，目标和反馈均为 DJI 原始编码器 0..8191。 */
#define LAUNCH_PITCH_DEFAULT_ENCODER   0U
#define LAUNCH_PITCH_KP                3.0f
#define LAUNCH_PITCH_KI                0.0f
#define LAUNCH_PITCH_KD                0.0f
#define LAUNCH_PITCH_CURRENT_LIMIT     6500.0f
#define LAUNCH_PITCH_DEADBAND_ENCODER  8.0f

#endif
