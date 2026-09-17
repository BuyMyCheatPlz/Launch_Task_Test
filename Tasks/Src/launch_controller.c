#include "launch_controller.h"
#include "launch_motor_bus.h"
#include "launch_fire_planner.h"
#include "config.h"
#include "cmsis_os2.h"
#include <stdlib.h>
#include <string.h>

#define UART_RX_SIZE 64U
#define VOFA_CHANNEL_COUNT 8U
#define VOFA_TX_SIZE (VOFA_CHANNEL_COUNT * sizeof(float) + 4U)
#define VOFA_TX_TIMEOUT_MS 100U
typedef struct { float kp, ki, kd, integral, previous_error, filtered_speed; uint8_t filter_ready; } SpeedPid_t;
typedef struct { volatile float speed_rpm; volatile uint32_t bullet_count, interval_ms; volatile uint8_t start_request, stop_request; } Command_t;

static UART_HandleTypeDef *launch_uart;
static uint8_t uart_rx[UART_RX_SIZE];
static uint8_t vofa_tx_buffer[VOFA_TX_SIZE];
static volatile uint8_t vofa_tx_busy;
static uint32_t vofa_tx_start_ms;
static uint8_t uart_rx_started;
static char line[40]; static uint8_t line_length;
static volatile Command_t command = { LAUNCH_DEFAULT_SPEED_RPM, LAUNCH_DEFAULT_BULLET_COUNT, LAUNCH_DEFAULT_SINGLE_INTERVAL_MS, 0U, 0U };
static SpeedPid_t left_pid = { LAUNCH_M3508_KP, LAUNCH_M3508_KI, LAUNCH_M3508_KD };
static SpeedPid_t right_pid = { LAUNCH_M3508_KP, LAUNCH_M3508_KI, LAUNCH_M3508_KD };
static SpeedPid_t feeder_pid = { LAUNCH_M2006_KP, LAUNCH_M2006_KI, 0.0f };
static volatile uint8_t active; static volatile uint32_t completed_bullets;
static volatile float actual_left_rpm, actual_right_rpm, feeder_deg, feeder_target_deg;
static LaunchFirePlanner_t fire_planner;
static volatile uint8_t jam_latched, jam_retreat_active;
#if (LAUNCH_ENABLE_JAM_PROTECTION != 0U)
static uint32_t jam_stall_since_ms;
#endif
static float jam_retreat_target_deg;

/* CubeMX 创建的 16 深度 uint16_t 队列：UART 中断只投递数据，
 * 命令解析在发射任务上下文完成，避免在中断优先级内执行字符串转换。 */
extern osMessageQueueId_t transportHandle;

static float clamp(float value, float limit) { if (value > limit) return limit; if (value < -limit) return -limit; return value; }
static float positive_progress(float value) { return (value > 0.0f) ? value : 0.0f; }
static int16_t pid_update(SpeedPid_t *pid, float target, float actual, float alpha, float limit)
{
    float error, output;
    if (pid->filter_ready == 0U) { pid->filtered_speed = actual; pid->filter_ready = 1U; }
    else pid->filtered_speed += alpha * (actual - pid->filtered_speed);
    error = target - pid->filtered_speed;
    pid->integral = clamp(pid->integral + error * pid->ki * 0.001f, limit);
    output = pid->kp * error + pid->integral + pid->kd * (error - pid->previous_error);
    pid->previous_error = error;
    return (int16_t)clamp(output, limit);
}
static void pid_reset(SpeedPid_t *pid) { pid->integral = 0.0f; pid->previous_error = 0.0f; pid->filter_ready = 0U; }
static void set_all_zero(void)
{
    LaunchMotorBus_SetCommand(LAUNCH_LEFT_FLYWHEEL_CAN, LAUNCH_LEFT_FLYWHEEL_ID, 0);
    LaunchMotorBus_SetCommand(LAUNCH_RIGHT_FLYWHEEL_CAN, LAUNCH_RIGHT_FLYWHEEL_ID, 0);
    LaunchMotorBus_SetCommand(LAUNCH_FEEDER_CAN, LAUNCH_FEEDER_ID, 0);
}
static void begin_rx(void)
{
    HAL_StatusTypeDef rx_status;
    if (launch_uart == 0) return;
    __HAL_UART_CLEAR_OREFLAG(launch_uart);
    __HAL_UART_CLEAR_IDLEFLAG(launch_uart);
    rx_status = HAL_UARTEx_ReceiveToIdle_DMA(launch_uart, uart_rx, sizeof(uart_rx));
    if (rx_status != HAL_OK)
    {
        (void)HAL_UART_AbortReceive(launch_uart);
        __HAL_UART_CLEAR_OREFLAG(launch_uart);
        __HAL_UART_CLEAR_IDLEFLAG(launch_uart);
        rx_status = HAL_UARTEx_ReceiveToIdle_DMA(launch_uart, uart_rx,
                                                 sizeof(uart_rx));
    }
    if ((rx_status == HAL_OK) && (launch_uart->hdmarx != 0)) __HAL_DMA_DISABLE_IT(launch_uart->hdmarx, DMA_IT_HT);
}

static void recover_tx(void)
{
    if (launch_uart == 0) return;
    CLEAR_BIT(launch_uart->Instance->CR3, USART_CR3_DMAT);
    if (launch_uart->hdmatx != 0) (void)HAL_DMA_Abort(launch_uart->hdmatx);
    launch_uart->TxXferCount = 0U;
    launch_uart->gState = HAL_UART_STATE_READY;
    launch_uart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_OREFLAG(launch_uart);
    __HAL_UART_CLEAR_IDLEFLAG(launch_uart);
    vofa_tx_busy = 0U;
}
/* U4/VOFA 文本协议：兼容大小写，参数可用 =、: 或空格分隔。
 * 例如：start、RUN、speed=6000、RPM : 6000、count 10、period=80。 */
static void normalize_command(void)
{
    uint8_t begin = 0U;
    uint8_t end = line_length;
    uint8_t out = 0U;
    while ((begin < end) && ((line[begin] == ' ') || (line[begin] == '\t')))
        ++begin;
    while ((end > begin) && ((line[end - 1U] == ' ') ||
                              (line[end - 1U] == '\t')))
        --end;
    while (begin < end)
    {
        char c = line[begin++];
        if ((c >= 'a') && (c <= 'z')) c = (char)(c - ('a' - 'A'));
        line[out++] = c;
    }
    line[out] = '\0';
}

static char *split_command_value(void)
{
    char *cursor = line;
    while ((*cursor != '\0') && (*cursor != '=') && (*cursor != ':') &&
           (*cursor != ' ') && (*cursor != '\t'))
        ++cursor;
    if (*cursor == '\0') return cursor;
    *cursor++ = '\0';
    while ((*cursor == '=') || (*cursor == ':') || (*cursor == ' ') ||
           (*cursor == '\t'))
        ++cursor;
    return cursor;
}

static uint8_t command_is(const char *name, const char *alias1,
                          const char *alias2)
{
    return (uint8_t)((strcmp(line, name) == 0) || (strcmp(line, alias1) == 0) ||
                     (strcmp(line, alias2) == 0));
}

static void process_line(void)
{
    char *end;
    char *value;
    float speed;
    unsigned long count, interval;
    normalize_command();
    value = split_command_value();
    if (command_is("START", "RUN", "ON") != 0U) command.start_request = 1U;
    else if (command_is("STOP", "HALT", "OFF") != 0U) command.stop_request = 1U;
    else if (command_is("SPEED", "RPM", "FLYWHEEL") != 0U)
    {
        speed = strtof(value, &end);
        if ((*value != '\0') && (*end == '\0') && (speed >= 0.0f) &&
            (speed <= 9000.0f)) command.speed_rpm = speed;
    }
    else if (command_is("COUNT", "BULLET", "SHOTS") != 0U)
    {
        count = strtoul(value, &end, 10);
        if ((*value != '\0') && (*end == '\0') && (count > 0U) &&
            (count <= 1000U)) command.bullet_count = (uint32_t)count;
    }
    else if (command_is("INTERVAL", "PERIOD", "GAP") != 0U)
    {
        interval = strtoul(value, &end, 10);
        if ((*value != '\0') && (*end == '\0') && (interval >= 1U) &&
            (interval <= 10000U)) command.interval_ms = (uint32_t)interval;
    }
    line_length = 0U;
}

void LaunchController_Init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2, UART_HandleTypeDef *uart)
{
    launch_uart = uart; line_length = 0U; LaunchMotorBus_Init(can1, can2);
}
void LaunchController_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t i;
    if ((huart != launch_uart) || (size > UART_RX_SIZE)) return;
    for (i = 0U; i < size; ++i)
    {
        uint16_t byte = uart_rx[i];
        /* 零等待时间可在中断中安全调用；队列满时直接丢弃，绝不阻塞 UART 中断。 */
        (void)osMessageQueuePut(transportHandle, &byte, 0U, 0U);
    }
    /* 接收空闲也视为一条命令结束，兼容不发送 CR/LF 的 VOFA 控件。 */
    {
        uint16_t delimiter = '\n';
        (void)osMessageQueuePut(transportHandle, &delimiter, 0U, 0U);
    }
    begin_rx();
}
void LaunchController_UartError(UART_HandleTypeDef *huart) { if (huart == launch_uart) { line_length = 0U; recover_tx(); (void)HAL_UART_AbortReceive(huart); begin_rx(); } }

/* 强定义覆盖 main.c 中 CubeMX 自动生成的弱任务入口。 */
void launch(void *argument) { LaunchController_Task(argument); }
void vofa(void *argument) { LaunchController_VofaTask(argument); }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    LaunchController_UartRxEvent(huart, size);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    LaunchController_UartError(huart);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == launch_uart) vofa_tx_busy = 0U;
}

static void process_transport_queue(void)
{
    uint16_t byte;
    while (osMessageQueueGet(transportHandle, &byte, 0U, 0U) == osOK)
    {
        char c = (char)byte;
        if ((c == '\r') || (c == '\n'))
        {
            if (line_length != 0U) process_line();
        }
        else if (line_length < (sizeof(line) - 1U))
        {
            line[line_length++] = c;
        }
        else
        {
            /* 命令过长则丢弃，等待下一个分隔符后重新同步。 */
            line_length = 0U;
        }
    }
}

void LaunchController_Task(void *argument)
{
    const LaunchMotorFeedback_t *left, *right, *feeder; uint32_t start_ms = 0U;
    uint32_t diff_stable_since_ms = 0U;
    int32_t last_encoder = 0, accumulator = 0; uint8_t encoder_ready = 0U;
    uint32_t wake_tick;
    (void)argument;
    if (uart_rx_started == 0U) { begin_rx(); uart_rx_started = 1U; }
    wake_tick = osKernelGetTickCount();
    for (;;) {
        uint32_t now = HAL_GetTick(); float feeder_speed = 0.0f;
        uint8_t flywheel_ready;
        uint8_t feeder_enable;
        float left_target_rpm;
        float right_target_rpm;
        process_transport_queue();
        left = LaunchMotorBus_Feedback(LAUNCH_LEFT_FLYWHEEL_CAN, LAUNCH_LEFT_FLYWHEEL_ID);
        right = LaunchMotorBus_Feedback(LAUNCH_RIGHT_FLYWHEEL_CAN, LAUNCH_RIGHT_FLYWHEEL_ID);
        feeder = LaunchMotorBus_Feedback(LAUNCH_FEEDER_CAN, LAUNCH_FEEDER_ID);
        flywheel_ready = (uint8_t)((left != 0) && (left->online != 0U) &&
                                   (right != 0) && (right->online != 0U));
        if (command.stop_request != 0U) { command.stop_request = 0U; active = 0U; jam_retreat_active = 0U; diff_stable_since_ms = 0U; set_all_zero(); pid_reset(&left_pid); pid_reset(&right_pid); pid_reset(&feeder_pid); encoder_ready = 0U; fire_planner.initialized = 0U; }
        if (command.start_request != 0U) { command.start_request = 0U; active = 1U; jam_latched = 0U; jam_retreat_active = 0U;
#if (LAUNCH_ENABLE_JAM_PROTECTION != 0U)
            jam_stall_since_ms = 0U;
#endif
            diff_stable_since_ms = 0U; completed_bullets = 0U; start_ms = now; encoder_ready = 0U; fire_planner.initialized = 0U; }
        if (active != 0U) {
            /* 摩擦轮任意一侧无反馈时，禁止推进拨盘。 */
            feeder_enable = (uint8_t)((flywheel_ready != 0U) &&
                ((now - start_ms) >= LAUNCH_FLYWHEEL_SPINUP_MS));
            if (flywheel_ready == 0U) { start_ms = now; diff_stable_since_ms = 0U; }
            left_target_rpm = LAUNCH_LEFT_FLYWHEEL_DIR * command.speed_rpm;
            right_target_rpm = LAUNCH_RIGHT_FLYWHEEL_DIR * command.speed_rpm;
#if (LAUNCH_ENABLE_FLYWHEEL_DIFF_LOOP != 0U)
            if (flywheel_ready != 0U)
            {
                float left_forward_rpm = LAUNCH_LEFT_FLYWHEEL_DIR *
                    (float)left->speed_rpm;
                float right_forward_rpm = LAUNCH_RIGHT_FLYWHEEL_DIR *
                    (float)right->speed_rpm;
                float correction_rpm = clamp((left_forward_rpm - right_forward_rpm) *
                    LAUNCH_FLYWHEEL_DIFF_KP,
                    LAUNCH_FLYWHEEL_DIFF_MAX_CORRECTION_RPM);
                float diff_abs_rpm = left_forward_rpm - right_forward_rpm;
                if (diff_abs_rpm < 0.0f) diff_abs_rpm = -diff_abs_rpm;
                /* 左侧较快时降低左目标、提高右目标；反之亦然。 */
                left_target_rpm = LAUNCH_LEFT_FLYWHEEL_DIR *
                    (command.speed_rpm - correction_rpm);
                right_target_rpm = LAUNCH_RIGHT_FLYWHEEL_DIR *
                    (command.speed_rpm + correction_rpm);
                if ((diff_abs_rpm <= LAUNCH_FLYWHEEL_DIFF_READY_RPM) &&
                    (left_forward_rpm > 0.0f) && (right_forward_rpm > 0.0f))
                {
                    if (diff_stable_since_ms == 0U) diff_stable_since_ms = now;
                    if ((now - diff_stable_since_ms) < LAUNCH_FLYWHEEL_DIFF_READY_MS)
                        feeder_enable = 0U;
                }
                else
                {
                    diff_stable_since_ms = 0U;
                    feeder_enable = 0U;
                }
            }
            else feeder_enable = 0U;
#endif
            if ((jam_retreat_active == 0U) && (left != 0) && (left->online != 0U)) { actual_left_rpm = (float)left->speed_rpm; LaunchMotorBus_SetCommand(LAUNCH_LEFT_FLYWHEEL_CAN, LAUNCH_LEFT_FLYWHEEL_ID, pid_update(&left_pid, left_target_rpm, actual_left_rpm, LAUNCH_M3508_FILTER_ALPHA, LAUNCH_M3508_CURRENT_LIMIT)); }
            if ((jam_retreat_active == 0U) && (right != 0) && (right->online != 0U)) { actual_right_rpm = (float)right->speed_rpm; LaunchMotorBus_SetCommand(LAUNCH_RIGHT_FLYWHEEL_CAN, LAUNCH_RIGHT_FLYWHEEL_ID, pid_update(&right_pid, right_target_rpm, actual_right_rpm, LAUNCH_M3508_FILTER_ALPHA, LAUNCH_M3508_CURRENT_LIMIT)); }
            if ((feeder != 0) && (feeder->online != 0U)) {
                int32_t delta, raw = (int32_t)feeder->encoder;
                if (encoder_ready == 0U) { last_encoder = raw; accumulator = 0; feeder_deg = 0.0f; feeder_target_deg = 0.0f; encoder_ready = 1U; LaunchFirePlanner_Reset(&fire_planner, feeder_deg, now); }
                delta = raw - last_encoder; if (delta > 4096) delta -= 8192; if (delta < -4096) delta += 8192; accumulator += delta; last_encoder = raw;
                feeder_deg = (float)accumulator * 360.0f / (8192.0f * LAUNCH_FEEDER_GEAR_RATIO);
                if (jam_retreat_active != 0U)
                {
                    /* 退弹阶段只控制拨盘，摩擦轮已关闭。 */
                    float retreat_error = jam_retreat_target_deg - feeder_deg;
                    feeder_target_deg = jam_retreat_target_deg;
                    feeder_speed = clamp(retreat_error * LAUNCH_FEEDER_ANGLE_KP,
                                         LAUNCH_FEEDER_MAX_RPM);
                    LaunchMotorBus_SetCommand(LAUNCH_FEEDER_CAN, LAUNCH_FEEDER_ID,
                        pid_update(&feeder_pid, feeder_speed, (float)feeder->speed_rpm,
                                   LAUNCH_M2006_FILTER_ALPHA, LAUNCH_M2006_CURRENT_LIMIT));
                    if ((retreat_error < LAUNCH_FEEDER_DEADBAND_DEG) &&
                        (retreat_error > -LAUNCH_FEEDER_DEADBAND_DEG))
                    {
                        jam_retreat_active = 0U;
                        active = 0U;
                        set_all_zero();
                    }
                }
                else
                {
                    float feeder_progress_deg;
                    float target_progress_deg;
                    float target_error;
                    LaunchFirePlanner_Update(&fire_planner, feeder_deg, now,
                        command.interval_ms, command.bullet_count,
                        feeder_enable);
                    feeder_target_deg = fire_planner.target_deg;
                    target_error = feeder_target_deg - feeder_deg;
                    feeder_progress_deg = positive_progress(LAUNCH_FEEDER_DIR *
                        (feeder_deg - fire_planner.start_deg));
                    target_progress_deg = positive_progress(LAUNCH_FEEDER_DIR *
                        (feeder_target_deg - fire_planner.start_deg));
                    if (fire_planner.issued_count >= command.bullet_count)
                    {
                        /* 目标发数已下发完成后只做位置保持，避免锁相环前馈继续推拨盘。 */
                        feeder_speed = clamp(target_error * LAUNCH_FEEDER_ANGLE_KP,
                                             LAUNCH_FEEDER_MAX_RPM);
                    }
                    else
                    {
                        feeder_speed = LaunchFirePlanner_TargetSpeedRpm(&fire_planner,
                            feeder_deg, command.interval_ms);
                    }
                    LaunchMotorBus_SetCommand(LAUNCH_FEEDER_CAN, LAUNCH_FEEDER_ID,
                        pid_update(&feeder_pid, feeder_speed, (float)feeder->speed_rpm,
                                   LAUNCH_M2006_FILTER_ALPHA, LAUNCH_M2006_CURRENT_LIMIT));
                    completed_bullets = (uint32_t)(feeder_progress_deg /
                        LAUNCH_FEEDER_STEP_DEG);
                    feeder_target_deg = fire_planner.start_deg +
                        LAUNCH_FEEDER_DIR * target_progress_deg;
#if (LAUNCH_ENABLE_JAM_PROTECTION != 0U)
                    if (((target_error >= LAUNCH_JAM_MIN_ERROR_DEG) ||
                         (target_error <= -LAUNCH_JAM_MIN_ERROR_DEG)) &&
                        ((feeder->speed_rpm <= LAUNCH_JAM_ZERO_SPEED_RPM) &&
                         (feeder->speed_rpm >= -LAUNCH_JAM_ZERO_SPEED_RPM)))
                    {
                        if (jam_stall_since_ms == 0U) jam_stall_since_ms = now;
                        if ((now - jam_stall_since_ms) >= LAUNCH_JAM_STALL_MS)
                        {
                            jam_latched = 1U;
                            jam_retreat_active = 1U;
                            jam_retreat_target_deg = feeder_deg -
                                LAUNCH_FEEDER_DIR * (float)LAUNCH_JAM_REVERSE_BULLETS *
                                LAUNCH_FEEDER_STEP_DEG;
                            jam_stall_since_ms = 0U;
                            LaunchMotorBus_SetCommand(LAUNCH_LEFT_FLYWHEEL_CAN,
                                LAUNCH_LEFT_FLYWHEEL_ID, 0);
                            LaunchMotorBus_SetCommand(LAUNCH_RIGHT_FLYWHEEL_CAN,
                                LAUNCH_RIGHT_FLYWHEEL_ID, 0);
                        }
                    }
                    else jam_stall_since_ms = 0U;
#endif
                    if ((feeder_enable != 0U) &&
                        (fire_planner.issued_count >= command.bullet_count) &&
                        (completed_bullets >= command.bullet_count)) { active = 0U; diff_stable_since_ms = 0U; set_all_zero(); }
                }
            }
        }
        else set_all_zero();
        LaunchMotorBus_Service(now);
        wake_tick += LAUNCH_CONTROL_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}

void LaunchController_VofaTask(void *argument)
{
    float out[8]; (void)argument;
    uint32_t wake_tick = osKernelGetTickCount();
    for (;;) {
        uint32_t now = HAL_GetTick();
        float vofa_target_deg = feeder_target_deg;
        float vofa_actual_deg = feeder_deg;
        if (fire_planner.initialized != 0U)
        {
            vofa_target_deg = LAUNCH_FEEDER_DIR *
                (feeder_target_deg - fire_planner.start_deg);
            vofa_actual_deg = LAUNCH_FEEDER_DIR *
                (feeder_deg - fire_planner.start_deg);
        }
        /* I0=转速，I1=发弹数，I2=单发间隔，I3=状态，I4=完成数，
           I5=左摩擦轮 rpm，I6=本次目标角，I7=本次实际角。 */
        out[0] = command.speed_rpm; out[1] = (float)command.bullet_count; out[2] = (float)command.interval_ms;
        out[3] = (jam_retreat_active != 0U) ? 2.0f : (jam_latched != 0U) ? -1.0f : (float)active;
        out[4] = (float)completed_bullets; out[5] = actual_left_rpm; out[6] = vofa_target_deg; out[7] = vofa_actual_deg;
        /* TX DMA 避免低优先级遥测任务阻塞 1 ms 控制任务；缓冲区必须保持
           有效，直到 TxCplt 回调释放它。 */
        if ((vofa_tx_busy != 0U) && ((now - vofa_tx_start_ms) >= VOFA_TX_TIMEOUT_MS))
        {
            /* 任意一次 DMA/TC 中断异常都不能让 VOFA 永久停发。 */
            recover_tx();
        }
        if (vofa_tx_busy == 0U)
        {
            HAL_StatusTypeDef tx_status;
            static const uint8_t tail[4] = { 0x00U, 0x00U, 0x80U, 0x7FU };
            memcpy(vofa_tx_buffer, out, VOFA_CHANNEL_COUNT * sizeof(float));
            memcpy(&vofa_tx_buffer[VOFA_CHANNEL_COUNT * sizeof(float)], tail,
                   sizeof(tail));
            vofa_tx_busy = 1U;
            vofa_tx_start_ms = now;
            tx_status = HAL_UART_Transmit_DMA(launch_uart, vofa_tx_buffer,
                                              sizeof(vofa_tx_buffer));
            if (tx_status != HAL_OK)
                recover_tx();
        }
        wake_tick += LAUNCH_VOFA_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
