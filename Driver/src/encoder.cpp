/**
 ******************************************************************************
 * @file           : encoder.cpp
 * @brief          : 四路 AB 相编码器驱动实现（Driver 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "encoder.hpp"
#include "main.h"

namespace encoder {

/* Private variables ---------------------------------------------------------*/
namespace {

TIM_HandleTypeDef g_timer[kCount]; 
uint16_t g_previous[kCount];

/**
 * @brief 配置四路编码器引脚与复用
 * @details PA15/PB3 原本是 JTAG 引脚；关闭 JTAG，保留 SWD 下载与调试。
 *          TIM2 需要部分重映射才能落到 PA15/PB3。
 */
void configure_pins() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    // PA15/PB3 原本是 JTAG 引脚；关闭 JTAG，保留 SWD 下载与调试。
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_AFIO_REMAP_TIM2_PARTIAL_2();

    GPIO_InitTypeDef pins{};
    pins.Mode = GPIO_MODE_INPUT;
    pins.Pull = GPIO_PULLUP;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;

    pins.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &pins); // M1: TIM2_CH1
    pins.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &pins); // M1: TIM2_CH2
    pins.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &pins); // M2: TIM3_CH1/CH2
    pins.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &pins); // M3: TIM5_CH1/CH2
    pins.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &pins); // M4: TIM4_CH1/CH2
}

/**
 * @brief 按 AB 相正交编码器模式初始化一路定时器
 * @param timer 定时器句柄（输出）
 * @param instance 定时器实例（TIM2/3/4/5）
 */
void configure_timer(TIM_HandleTypeDef &timer, TIM_TypeDef *instance) {
    timer.Instance = instance;
    timer.Init.Prescaler = 0;
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = 0xFFFF;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef config{};
    config.EncoderMode = TIM_ENCODERMODE_TI12;
    config.IC1Polarity = TIM_ICPOLARITY_RISING;
    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC1Prescaler = TIM_ICPSC_DIV1;
    config.IC1Filter = 4;
    config.IC2Polarity = TIM_ICPOLARITY_RISING;
    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC2Prescaler = TIM_ICPSC_DIV1;
    config.IC2Filter = 4;
    if (HAL_TIM_Encoder_Init(&timer, &config) != HAL_OK ||
        HAL_TIM_Encoder_Start(&timer, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(&timer, 0);
}

} // namespace

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化四路编码器
 * @details 配置引脚和定时器，并把计数基准清零。
 */
void init() {
    configure_pins();
    configure_timer(g_timer[0], TIM2); // M1: PA15/PB3
    configure_timer(g_timer[1], TIM3); // M2: PA6/PA7
    configure_timer(g_timer[2], TIM5); // M3: PA0/PA1
    configure_timer(g_timer[3], TIM4); // M4: PB6/PB7
    for (unsigned i = 0; i < kCount; ++i) {
        g_previous[i] = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&g_timer[i]));
    }
}

/**
 * @brief 读一路编码器的计数增量
 * @details int16_t 差值天然处理 16 位回绕；读完即前移内部基准。
 * @param channel 通道号 0..3
 * @return 带符号增量；越界返回 0
 */
int delta_ticks(unsigned channel) {
    if (channel >= kCount) {
        return 0;
    }
    const uint16_t count = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&g_timer[channel]));
    // 16 位计数器回绕用 int16_t 差值天然处理。
    const int delta = static_cast<int16_t>(count - g_previous[channel]);
    g_previous[channel] = count;
    return delta;
}

} // namespace encoder
