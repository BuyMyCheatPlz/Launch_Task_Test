#include "launch_motor_bus.h"
#include "config.h"
#include <string.h>

static CAN_HandleTypeDef *buses[2];
static LaunchMotorFeedback_t feedback[2][8];
static int16_t command[2][8];

static CAN_HandleTypeDef *bus_for(uint8_t index) { return (index == 1U) ? buses[0] : (index == 2U) ? buses[1] : 0; }
static void put_be(uint8_t *p, int16_t value) { p[0] = (uint8_t)((uint16_t)value >> 8); p[1] = (uint8_t)value; }

void LaunchMotorBus_Init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2)
{
    CAN_FilterTypeDef filter = {0};
    buses[0] = can1; buses[1] = can2;
    memset(feedback, 0, sizeof(feedback)); memset(command, 0, sizeof(command));
    filter.FilterActivation = ENABLE; filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT; filter.FilterIdHigh = 0; filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0; filter.FilterMaskIdLow = 0; filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank = 0; filter.SlaveStartFilterBank = 14;
    (void)HAL_CAN_ConfigFilter(can1, &filter);
    filter.FilterBank = 14; (void)HAL_CAN_ConfigFilter(can2, &filter);
    (void)HAL_CAN_Start(can1); (void)HAL_CAN_Start(can2);
    (void)HAL_CAN_ActivateNotification(can1, CAN_IT_RX_FIFO0_MSG_PENDING);
    (void)HAL_CAN_ActivateNotification(can2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void LaunchMotorBus_SetCommand(uint8_t can_index, uint8_t motor_id, int16_t current)
{
    if ((can_index >= 1U) && (can_index <= 2U) && (motor_id >= 1U) && (motor_id <= 8U)) command[can_index - 1U][motor_id - 1U] = current;
}

const LaunchMotorFeedback_t *LaunchMotorBus_Feedback(uint8_t can_index, uint8_t motor_id)
{
    if ((can_index < 1U) || (can_index > 2U) || (motor_id < 1U) || (motor_id > 8U)) return 0;
    return &feedback[can_index - 1U][motor_id - 1U];
}

static void send_group(CAN_HandleTypeDef *bus, uint32_t id, const int16_t values[4])
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8]; uint32_t mailbox; uint8_t i;
    if ((bus == 0) || (HAL_CAN_GetTxMailboxesFreeLevel(bus) == 0U)) return;
    header.StdId = id; header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    for (i = 0; i < 4U; ++i) put_be(&data[i * 2U], values[i]);
    (void)HAL_CAN_AddTxMessage(bus, &header, data, &mailbox);
}

void LaunchMotorBus_Service(uint32_t now_ms)
{
    uint8_t b, id;
    for (b = 0; b < 2U; ++b) {
        int16_t low[4], high[4];
        for (id = 0; id < 4U; ++id) { low[id] = command[b][id]; high[id] = command[b][id + 4U]; }
        send_group(buses[b], 0x200U, low); send_group(buses[b], 0x1FFU, high);
        for (id = 0; id < 8U; ++id) if ((feedback[b][id].online != 0U) && ((now_ms - feedback[b][id].last_feedback_ms) > LAUNCH_MOTOR_OFFLINE_MS)) feedback[b][id].online = 0U;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header; uint8_t data[8]; uint8_t b, id;
    if ((hcan != buses[0]) && (hcan != buses[1])) return;
    b = (hcan == buses[0]) ? 0U : 1U;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if ((header.IDE != CAN_ID_STD) || (header.DLC != 8U) || (header.StdId < 0x201U) || (header.StdId > 0x208U)) continue;
        id = (uint8_t)(header.StdId - 0x201U);
        feedback[b][id].encoder = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
        feedback[b][id].speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        feedback[b][id].current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        feedback[b][id].last_feedback_ms = HAL_GetTick(); feedback[b][id].online = 1U;
    }
}
