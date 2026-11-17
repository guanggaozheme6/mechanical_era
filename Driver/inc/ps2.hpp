/**
 ******************************************************************************
 * @file           : ps2.hpp
 * @brief          : PS2 接收器的 SPI 通信、轮询与超时（Driver 层）
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ps2_protocol.hpp"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PS2 通信诊断统计
 * @note 只用于观察，不作为控制逻辑的输入。
 */
struct Ps2Diagnostics {
    uint32_t polls = 0;           // 轮询次数
    uint32_t valid_frames = 0;    // 三重校验通过的帧数
    uint32_t invalid_frames = 0;  // 校验失败的帧数
    uint32_t spi_errors = 0;      // HAL_SPI 返回非 OK 的次数
    uint32_t input_changes = 0;   // 按键/摇杆发生变化的次数
    uint32_t last_valid_ms = 0;   // 最近一次有效帧的时刻
    uint32_t last_hal_status = 0; // 最近一次 HAL 状态码
    uint32_t raw_length = 0;      // 最近一帧实际收到的字节数
    uint32_t config_attempts = 0; // 进入模拟模式的尝试次数
    uint8_t mode = 0;             // 最近一次的模式字（0x41/0x73/0x79）
    uint8_t raw[24] = {};         // 最近一帧原始字节，排查时序用
};

/* Exported variables --------------------------------------------------------*/

extern Ps2Diagnostics ps2_debug;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化接收器
 * @details 记下 SPI 句柄、把片选置高、使能 SPI 让 SCK 落到空闲高电平，再等 300 ms
 *          让接收器上电稳定。
 * @param spi 已初始化的 SPI 句柄（Mode 3 / LSB / 分频见 CubeMX）
 */
void ps2_init(SPI_HandleTypeDef &spi);

/**
 * @brief 推进一次轮询（主循环调用，不在中断里）
 * @details 20 ms 一次；整帧保持片选低、字节之间留 20 us 空档；收到第 2 字节后按模式字
 *          决定本帧还要收多少；没连上或不处于模拟模式时每 1 s 重协商一次。
 */
void ps2_update();

/**
 * @brief 取最新一帧
 * @return 超过 100 ms 没有有效报文时返回断线、中位、无按键的空帧
 */
Ps2Frame ps2_read();
