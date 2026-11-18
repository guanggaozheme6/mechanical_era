/**
 ******************************************************************************
 * @file           : hal_handles.h
 * @brief          : HAL 外设句柄声明
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported variables --------------------------------------------------------*/

extern SPI_HandleTypeDef hspi2;   // PS2 接收器
extern TIM_HandleTypeDef htim1;   // 四路电机 PWM
extern TIM_HandleTypeDef htim8;   // 四路舵机 PWM
extern UART_HandleTypeDef huart2; // 备用串口
extern UART_HandleTypeDef huart3; // 状态上报串口

#ifdef __cplusplus
}
#endif
