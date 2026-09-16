#ifndef LAUNCH_CONTROLLER_H
#define LAUNCH_CONTROLLER_H

#include "main.h"

void LaunchController_Init(CAN_HandleTypeDef *can1, CAN_HandleTypeDef *can2,
                           UART_HandleTypeDef *uart);
void LaunchController_Task(void *argument);
void LaunchController_VofaTask(void *argument);
void LaunchController_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void LaunchController_UartError(UART_HandleTypeDef *huart);

#endif
