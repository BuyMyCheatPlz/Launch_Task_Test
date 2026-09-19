# UART4 发射任务

UART4（115200, 8-N-1）同时承载 VOFA telemetry 和 ASCII 控制命令，实际引脚为 `PC10=TX`、`PC11=RX`。每个命令以换行结束；UART 空闲也会提交一条命令。VOFA 打印使用 UART4 TX DMA 持续发送。

| 命令 | 作用 |
| --- | --- |
| `SPEED=6000` | 设置两颗 M3508 的目标转速（0–9000 rpm） |
| `COUNT=10` | 设置发弹数量（1–1000；运行中修改会更新本次目标） |
| `INTERVAL=80` | 设置单发模式的相邻两发间隔（1–10000 ms） |
| `START` / `START=1` | 锁存一次发射请求：摩擦轮预转 500 ms 后，M2006 拨盘推进 `COUNT` 发并自动停机 |
| `START=0` | 手动释放 START 锁存，允许下一次 `START` 再次触发 |
| `STOP` | 立即将三颗电机电流置零，并释放 START 锁存 |
| `PITCH=4096` | 当 `LAUNCH_ENABLE_PITCH_M2006=1U` 时，控制 CAN2 ID1 的 pitch 轴 2006 摆到指定原始编码器位置（0–8191） |
| `PITCHSTOP` | 当 pitch 轴宏开启时，取消 pitch 轴输出；兼容 `PITCHOFF`、`PITCH_STOP` |

命令不区分大小写，参数分隔符可使用 `=`、`:` 或空格；例如 `speed=6000`、`RPM : 6000`、`count 10`、`period=80` 都有效。启停还兼容 `RUN`/`ON` 与 `HALT`/`OFF`。`START` 使用单锁存逻辑：一次任务运行期间重复发送 `START` 或 `START=1` 不会重复触发；正常完成全部发弹、收到 `START=0` 或收到 `STOP` 后会释放锁存。锁存释放后再次收到 `START` 或 `START=1` 就会开始下一次任务，因此上位机不应在任务完成后仍持续周期发送 `START=1`。

VOFA 默认使用 justfloat 接收 8 个基础通道：速度设定、发弹数、单发间隔、运行标志、已完成发数、左摩擦轮转速、本次拨盘目标角、本次拨盘实际角。发送 `SPEED=`、`COUNT=`、`INTERVAL=` 即可在线修改参数。开启 pitch 轴宏后会追加 pitch 轴编码器值；CAN2 诊断数据默认保留在底层但不打印。

## 任务周期

控制任务和 VOFA 打印任务都使用绝对周期调度，不会因为本轮代码执行耗时而累计漂移：

```c
#define LAUNCH_CONTROL_PERIOD_MS       1U
#define LAUNCH_VOFA_PERIOD_MS          20U
```

## VOFA 打印通道

| 通道 | 名称 | 单位 | 含义 |
| --- | --- | --- | --- |
| I0 | `speed_set` | rpm | 两颗摩擦轮的目标转速设定值 |
| I1 | `count_set` | 发 | 本次目标发弹数 |
| I2 | `interval_set` | ms | 单发模式相邻两发的设定间隔；锁相环模式下为每发周期 |
| I3 | `launch_state` | - | `0` 空闲；`1` 正常发射；`2` 卡弹退弹中；`-1` 卡弹退弹完成并锁止，等待下一次 `START` |
| I4 | `completed_count` | 发 | 由拨盘实际输出角度换算的已完成发数 |
| I5 | `left_flywheel_rpm` | rpm | 左摩擦轮实际反馈转速 |
| I6 | `feeder_target_deg` | ° | 本次 `START` 后的拨盘输出轴目标角度；1 发默认应接近 40° |
| I7 | `feeder_actual_deg` | ° | 本次 `START` 后的拨盘输出轴实际角度；用于检查单发是否过冲 |
| I8 | `pitch_encoder` | tick | 仅 `LAUNCH_ENABLE_PITCH_M2006=1U` 时存在；CAN2 ID1 pitch 轴 2006 的 DJI 原始编码器值（0–8191） |

CAN2 诊断统计数据会保留在 `LaunchMotorBus_Diag()` 中，但当前 `LAUNCH_ENABLE_CAN2_DIAG=0U`，不会追加到 VOFA。需要再次排查 CAN2 线时，把该宏临时改为 `1U`，会在基础通道和 pitch 通道之后追加 7 个 CAN2 诊断通道；如果 pitch 也开启，对应为：

| 通道 | 名称 | 单位 | 含义 |
| --- | --- | --- | --- |
| I9 | `can2_rx_count` | 帧 | CAN2 收到的总帧数；正常时应持续增加 |
| I10 | `can2_last_id` | - | CAN2 最近一次收到的标准帧 ID；2006 ID1 正常反馈应为 `0x201`，VOFA 中显示为十进制 `513` |
| I11 | `can2_ms_since_rx` | ms | 距离上次 CAN2 收包的时间；持续增大说明 CAN2 没有继续收到帧 |
| I12 | `can2_error_code` | - | HAL CAN 错误码；`0` 表示 HAL 当前未记录错误 |
| I13 | `can2_esr_flags` | - | CAN ESR 状态位：bit0=warning，bit1=passive，bit2=bus-off |
| I14 | `can2_tec` | - | CAN 发送错误计数；持续升高通常说明没有 ACK、线断、终端/波特率/电调供电异常 |
| I15 | `can2_rec` | - | CAN 接收错误计数；持续升高通常说明采样点、波特率、干扰或终端异常 |

## 启动条件

`START` 命令会立即被接收，但拨盘不会立刻动作。任务要求两颗摩擦轮在线，并等待 `LAUNCH_FLYWHEEL_SPINUP_MS` 预转完成；差速环开启时，还要等左右摩擦轮转速差连续稳定后，才允许拨盘发弹。拨盘电机不在线时不会发弹。

发弹完成后会把三颗电机电流全部置零，摩擦轮停止转动。

## 主要配置

`config.h` 的 `LAUNCH_ENABLE_PLL_FIRE_MODE` 决定拨盘规划方式：`1U` 使用原工程的锁相环连续推进；`0U` 使用单发计数规划，每一发由 `INTERVAL` 控制间隔。

卡弹保护由 `LAUNCH_ENABLE_JAM_PROTECTION` 控制。启用后，若拨盘目标误差持续大于 `LAUNCH_JAM_MIN_ERROR_DEG` 且反馈转速持续低于 `LAUNCH_JAM_ZERO_SPEED_RPM` 达 `LAUNCH_JAM_STALL_MS`，系统将关闭摩擦轮、反向退 `LAUNCH_JAM_REVERSE_BULLETS` 发，随后关闭全部电机；只能用下一次 `START` 解除锁止。VOFA I3：`1` 运行、`2` 退弹中、`-1` 已锁止、`0` 空闲。

电机接线、CAN 端口、CAN ID、方向、减速比、PID 与安全限制集中在 [Application_Config/Inc/config.h](Application_Config/Inc/config.h)。CAN 端口使用 `1` 表示 CAN1、`2` 表示 CAN2；DJI 电机 ID 允许 `1..8`。

方向校准宏为 `LAUNCH_LEFT_FLYWHEEL_DIR`、`LAUNCH_RIGHT_FLYWHEEL_DIR`、`LAUNCH_FEEDER_DIR` 与 `LAUNCH_PITCH_DIR`：需要反向时将对应值由 `1.0f` 改成 `-1.0f`。其中拨盘方向同时影响正向发弹和卡弹后的反向退弹。

可选 pitch 轴 2006 默认关闭；开启后才会编译 CAN2 ID1 的位置环、`PITCH=` 命令和 VOFA I8 通道。目标值与反馈值均为 DJI 原始编码器 `0..8191`，收到 `PITCH=目标编码器值` 后才会输出电流摆动，未收到指令时不会上电自动追默认位置：

```c
#define LAUNCH_ENABLE_PITCH_M2006      1U
#define LAUNCH_PITCH_CAN               2U
#define LAUNCH_PITCH_ID                1U
#define LAUNCH_PITCH_DIR               1.0f
#define LAUNCH_PITCH_KP                3.0f
#define LAUNCH_PITCH_CURRENT_LIMIT     6500.0f
#define LAUNCH_PITCH_DEADBAND_ENCODER  8.0f
```

如果 pitch 轴方向相反，把 `LAUNCH_PITCH_DIR` 改成 `-1.0f`。需要取消 pitch 输出时发送 `PITCHSTOP`。

每发弹丸对应的拨盘输出轴角度由 `LAUNCH_FEEDER_STEP_DEG` 控制，默认 `40.0f`（单位：°）。例如机构实测每发需要拨盘旋转 45° 时，改为：

```c
#define LAUNCH_FEEDER_STEP_DEG         45.0f
```

修改后，单发目标、锁相环连发相位、发弹完成计数，以及卡弹时按 `LAUNCH_JAM_REVERSE_BULLETS` 设置的退弹角度都会同步按新值计算。当前卡弹退弹量为 1 发。

单发到达目标后会进入位置保持，并用较低速度上限抑制重负载释放后的过冲：

```c
#define LAUNCH_FEEDER_HOLD_MAX_RPM     1200.0f
```

两颗 3508 摩擦轮可使用差速补偿环保持同向等效转速一致：

```c
#define LAUNCH_ENABLE_FLYWHEEL_DIFF_LOOP 1U
```

该功能当前默认开启。开启后，`LAUNCH_FLYWHEEL_DIFF_KP` 控制左右转速差转化为目标转速修正的比例，`LAUNCH_FLYWHEEL_DIFF_MAX_CORRECTION_RPM` 限制最大修正量。方向宏会先用于归一化实际转速，因此左右电机物理安装方向相反时也能正确比较。

差速环开启时，拨盘还会等待左右摩擦轮转速差稳定后才发弹：

```c
#define LAUNCH_FLYWHEEL_DIFF_READY_RPM 100.0f
#define LAUNCH_FLYWHEEL_DIFF_READY_MS  100U
```

差速稳定条件只控制“何时下发下一发目标”。某个 40° 步进一旦下发，即使运动过程中转速差短暂超过阈值，也会继续完成当前步进，不会清零规划器或在恢复稳定后重复追加一发。
