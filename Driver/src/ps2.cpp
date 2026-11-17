/**
 ******************************************************************************
 * @file           : ps2.cpp
 * @brief          : PS2 接收器 SPI 通信实现（Driver 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ps2.hpp"
#include <algorithm>

/* Exported variables --------------------------------------------------------*/

Ps2Diagnostics ps2_debug;

/* Private constants ---------------------------------------------------------*/
namespace {
constexpr uint32_t kPollMs = 20;    // 轮询周期
constexpr uint32_t kConfigMs = 1000; // 重协商间隔
constexpr uint32_t kTimeoutMs = 100; // 有效帧超时 → 断线
SPI_HandleTypeDef *receiver_spi = nullptr;
Ps2Frame latest_frame;
uint32_t last_poll = 0;
uint32_t last_config = 0;
uint32_t last_valid_ms = 0;

/**
 * @brief 忙等微秒延时
 * @details 用 DWT 周期计数器实现；只在 SPI 帧内用（字节间隙 20 us）。
 * @param us 延时时长（微秒）
 */
void delay_us(uint32_t us) {
    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while (static_cast<uint32_t>(DWT->CYCCNT - start) < cycles) {
    }
}

/**
 * @brief 一整帧的双向移位交换
 * @details 整帧保持 ATT 为低；字节间隔为未接线的 ACK 留出应答时间。逐字节调用
 *          HAL_SPI_TransmitReceive（不要一口气传 9 字节，帧内没有 ACK 空档）。
 * @param tx 发送缓冲
 * @param rx 接收缓冲（输出）
 * @param length 入：请求长度；出：实际收到的长度
 * @param polling true = 轮询帧（收到第 2 字节后按模式字动态决定长度）
 * @return HAL 状态；非 OK 时已经累加 spi_errors
 */
HAL_StatusTypeDef exchange(const uint8_t *tx, uint8_t (&rx)[24], std::size_t &length,
                           bool polling) {
    std::fill_n(rx, sizeof(rx), 0xFF);
    const std::size_t requested = length;
    std::size_t expected = requested;
    length = 0;
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);
    delay_us(20);
    HAL_StatusTypeDef status = HAL_OK;
    for (std::size_t i = 0; i < expected; ++i) {
        uint8_t out = tx[i];
        status = HAL_SPI_TransmitReceive(receiver_spi, &out, &rx[i], 1, 5);
        if (status != HAL_OK) {
            break;
        }
        ++length;
        if (polling && i == 1) {
            const auto packet_size = ps2_packet_size(rx[1]);
            expected = packet_size == 0 ? requested : packet_size;
        }
        delay_us(20);
    }
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    ps2_debug.last_hal_status = status;
    if (status != HAL_OK) {
        ++ps2_debug.spi_errors;
    }
    return status;
}

/**
 * @brief 把手柄切到模拟模式（红灯）
 * @details 三条命令：进配置、设定模拟+震动、退出配置。手柄中途休眠/换电池会退回
 *          数字模式，所以这里由 ps2_update 周期重试自愈。
 */
void configure_analog() {
    constexpr uint8_t commands[3][9] = {{0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
                                        {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00},
                                        {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A}};
    ++ps2_debug.config_attempts;
    for (const auto &command : commands) {
        uint8_t rx[24];
        std::size_t length = sizeof(command);
        const auto status = exchange(command, rx, length, false);
        HAL_Delay(10);
        if (status != HAL_OK) {
            break;
        }
    }
}
} // namespace

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化接收器
 * @details 片选置高、使能 SPI 让 SCK 落到空闲高电平、等 300 ms 上电稳定，并把轮询与
 *          协商计时器拨到"立刻到点"，让第一帧马上发出。
 * @param spi 已初始化的 SPI 句柄
 */
void ps2_init(SPI_HandleTypeDef &spi) {
    receiver_spi = &spi;
    latest_frame = {};
    last_valid_ms = 0;
    ps2_debug = {};
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    // 拉低 ATT 前确保 SPI 时钟处于高电平空闲状态。
    __HAL_SPI_ENABLE(&spi);
    HAL_Delay(300);
    last_poll = HAL_GetTick() - kPollMs;
    last_config = HAL_GetTick() - kConfigMs;
}

/**
 * @brief 推进一次轮询（20 ms 一次）
 * @details 交换一帧 → 三重校验 → 更新诊断统计 → 没连上或不在模拟模式时每 1 s 重协商。
 */
void ps2_update() {
    if (receiver_spi == nullptr) {
        return;
    }
    const uint32_t now = HAL_GetTick();
    if (static_cast<uint32_t>(now - last_poll) < kPollMs) {
        return;
    }
    last_poll = now;
    constexpr uint8_t poll[24] = {0x01, 0x42};
    uint8_t rx[24];
    std::size_t length = 9;
    const auto status = exchange(poll, rx, length, true);
    const auto frame = status == HAL_OK ? decode_ps2_frame(rx, length) : Ps2Frame{};
    ++ps2_debug.polls;
    ps2_debug.raw_length = static_cast<uint32_t>(length);
    std::copy_n(rx, sizeof(rx), ps2_debug.raw);
    ps2_debug.mode = frame.connected ? rx[1] : 0;
    if (frame.connected) {
        if (latest_frame.connected &&
            (frame.buttons != latest_frame.buttons || frame.left_x != latest_frame.left_x ||
             frame.left_y != latest_frame.left_y)) {
            ++ps2_debug.input_changes;
        }
        last_valid_ms = HAL_GetTick();
        ps2_debug.last_valid_ms = last_valid_ms;
        ++ps2_debug.valid_frames;
    } else {
        ++ps2_debug.invalid_frames;
    }
    latest_frame = frame;
    if ((!frame.connected || !frame.analog) &&
        static_cast<uint32_t>(now - last_config) >= kConfigMs) {
        last_config = now;
        configure_analog();
    }
}

/**
 * @brief 取最新一帧
 * @return 超过 100 ms 无有效报文时返回断线空帧
 */
Ps2Frame ps2_read() {
    if (static_cast<uint32_t>(HAL_GetTick() - last_valid_ms) >= kTimeoutMs) {
        return {};
    }
    return latest_frame;
}
