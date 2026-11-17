/**
 ******************************************************************************
 * @file           : encoder.hpp
 * @brief          : 四路 AB 相编码器的配置与计数读取（Driver 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace encoder {

/* Exported constants --------------------------------------------------------*/

/** @brief 编码器通道数 */
constexpr unsigned kCount = 4U;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 配置四路编码器
 * @details 配置 TIM2/3/4/5 的编码器模式
 *          初始化与计数器清零。
 */
void init();

/**
 * @brief 自上次调用以来的计数增量(motor.c)
 * @param channel 通道号 0..3
 * @return 计数增量；越界返回 0
 */
int delta_ticks(unsigned channel);

} // namespace encoder
