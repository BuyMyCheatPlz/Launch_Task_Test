# UART4 发射任务

UART4（115200, 8-N-1）同时承载 VOFA telemetry 和 ASCII 控制命令。每个命令以换行结束；UART 空闲也会提交一条命令。

| 命令 | 作用 |
| --- | --- |
| `SPEED=6000` | 设置两颗 M3508 的目标转速（0–9000 rpm） |
| `COUNT=10` | 设置发弹数量（1–1000；运行中修改会更新本次目标） |
| `INTERVAL=80` | 设置单发模式的相邻两发间隔（1–10000 ms） |
| `START` | 摩擦轮预转 500 ms 后，M2006 拨盘推进 `COUNT` 发并自动停机 |
| `STOP` | 立即将三颗电机电流置零 |

命令不区分大小写，参数分隔符可使用 `=`、`:` 或空格；例如 `speed=6000`、`RPM : 6000`、`count 10`、`period=80` 都有效。启停还兼容 `RUN`/`ON` 与 `HALT`/`OFF`。

VOFA 使用 justfloat 接收 8 个通道：速度设定、发弹数、单发间隔、运行标志、已完成发数、左摩擦轮转速、拨盘目标角、拨盘实际角。发送 `SPEED=`、`COUNT=`、`INTERVAL=` 即可在线修改参数。

## VOFA 打印通道

| 通道 | 名称 | 单位 | 含义 |
| --- | --- | --- | --- |
| I0 | `speed_set` | rpm | 两颗摩擦轮的目标转速设定值 |
| I1 | `count_set` | 发 | 本次目标发弹数 |
| I2 | `interval_set` | ms | 单发模式相邻两发的设定间隔；锁相环模式下为每发周期 |
| I3 | `launch_state` | - | `0` 空闲；`1` 正常发射；`2` 卡弹退弹中；`-1` 卡弹退弹完成并锁止，等待下一次 `START` |
| I4 | `completed_count` | 发 | 由拨盘实际输出角度换算的已完成发数 |
| I5 | `left_flywheel_rpm` | rpm | 左摩擦轮实际反馈转速 |
| I6 | `feeder_target_deg` | ° | 拨盘输出轴目标角度；卡弹时显示退弹目标 |
| I7 | `feeder_actual_deg` | ° | 拨盘输出轴实际累计角度 |

`config.h` 的 `LAUNCH_ENABLE_CONTINUOUS_PLL` 决定拨盘规划方式：`1U` 使用原工程的锁相环连续推进；`0U` 使用单发计数规划，每一发由 `INTERVAL` 控制间隔。

卡弹保护由 `LAUNCH_ENABLE_JAM_PROTECTION` 控制。启用后，若拨盘目标误差持续大于 `LAUNCH_JAM_MIN_ERROR_DEG` 且反馈转速持续低于 `LAUNCH_JAM_ZERO_SPEED_RPM` 达 `LAUNCH_JAM_STALL_MS`，系统将关闭摩擦轮、反向退 `LAUNCH_JAM_REVERSE_BULLETS` 发，随后关闭全部电机；只能用下一次 `START` 解除锁止。VOFA I3：`1` 运行、`2` 退弹中、`-1` 已锁止、`0` 空闲。

电机接线、CAN 端口、CAN ID、方向、减速比、PID 与安全限制集中在 [Application_Config/Inc/config.h](Application_Config/Inc/config.h)。CAN 端口使用 `1` 表示 CAN1、`2` 表示 CAN2；DJI 电机 ID 允许 `1..8`。

方向校准宏为 `LAUNCH_LEFT_FLYWHEEL_DIR`、`LAUNCH_RIGHT_FLYWHEEL_DIR` 与 `LAUNCH_FEEDER_DIR`：需要反向时将对应值由 `1.0f` 改成 `-1.0f`。其中拨盘方向同时影响正向发弹和卡弹后的反向退弹。

每发弹丸对应的拨盘输出轴角度由 `LAUNCH_FEEDER_STEP_DEG` 控制，默认 `40.0f`（单位：°）。例如机构实测每发需要拨盘旋转 45° 时，改为：

```c
#define LAUNCH_FEEDER_STEP_DEG         45.0f
```

修改后，单发目标、锁相环连发相位、发弹完成计数，以及卡弹时的“退 5 发”角度都会同步按新值计算。
