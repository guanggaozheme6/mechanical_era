/**
 ******************************************************************************
 * @file           : encoder.hpp
 * @brief          : 四路 AB 相编码器的配置与计数读取（Driver 层）
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
#include <cstdint>

// 只管「哪几个引脚、哪个定时器、这一周期转了多少计数」，不做闭环。
// 不判堵转、不做 PID、不认识电机 —— 那是 Device/motor 的职责。
// 引脚与定时器映射与实车接线绑定，属于驱动细节，不进任何算法。

namespace encoder {

/* Exported constants --------------------------------------------------------*/

/** @brief 编码器通道数（与四路电机一一对应） */
constexpr unsigned kCount = 4U;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 配置四路编码器
 * @details 含 PA15/PB3 的 JTAG 解除复用（保留 SWD 调试）、TIM2/3/4/5 的编码器模式
 *          初始化与计数器清零。
 * @warning 会关掉 JTAG，只保留 SWD；换板子接线时先确认调试口。
 */
void init();

/**
 * @brief 自上次调用以来的计数增量
 * @details 带符号（正 = 计数器递增方向）。每调用一次内部基准就前移，所以四路各自
 *          只在 motor 每周期取一次。
 * @param channel 通道号 0..3
 * @return 计数增量；越界返回 0
 */
int delta_ticks(unsigned channel);

} // namespace encoder
