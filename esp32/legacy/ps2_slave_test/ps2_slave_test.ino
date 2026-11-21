/**
 ******************************************************************************
 * @file           : ps2_slave_test.ino
 * @brief          : ESP32-C3 自环验证 PS2 从机协议（软件主机 + 硬件 SPI2 从机）
 *
 * 主机：GPIO 软件 bit-bang，Mode 3 / LSB first，完全模仿 mechanical_era 的
 *       Driver/src/ps2.cpp：整帧 CS 低、字节间 20 us、20 ms 一轮。
 * 从机：SPI2 硬件 slave，回 0xFF 0x73 0x5A + 6 字节数据。
 *
 * 接线：默认内部回环（GPIO matrix），不需要杜邦线。
 *       若内部回环不通，把 LOOPBACK_INTERNAL 改成 0，外部接 4 根线：
 *         GPIO4 (host SCK)  -> GPIO0 (slave SCK)
 *         GPIO6 (host MOSI) -> GPIO1 (slave MOSI)
 *         GPIO5 (slave MISO) -> GPIO3 (host MISO)
 *         GPIO7 (host CS)   -> GPIO10 (slave CS)
 ******************************************************************************
 */

#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Private defines -----------------------------------------------------------*/

#define LOOPBACK_INTERNAL 1

#if LOOPBACK_INTERNAL
static const int HOST_SCK = 4, HOST_MOSI = 6, HOST_CS = 7, HOST_MISO = 5;
static const int SLV_SCK = 4, SLV_MOSI = 6, SLV_CS = 7, SLV_MISO = 5;
#else
static const int HOST_SCK = 4, HOST_MOSI = 6, HOST_CS = 7, HOST_MISO = 3;
static const int SLV_SCK = 0, SLV_MOSI = 1, SLV_CS = 10, SLV_MISO = 5;
#endif

/* Private variables ---------------------------------------------------------*/

static uint8_t s_tx[24];
static uint8_t s_rx[24];
static uint8_t s_last_rx[24];
static volatile int s_rx_len = 0;
static volatile uint32_t s_frames = 0;
static volatile uint32_t s_errors = 0;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 主机侧 bit-bang 一拍：Mode 3，先放数据、拉低 SCK、再拉高采样
 * @param out 主机要发的字节
 * @param in  主机读回的字节（引用）
 */
static void host_byte(uint8_t out, uint8_t &in) {
    in = 0;
    for (int b = 0; b < 8; ++b) {
        gpio_set_level((gpio_num_t)HOST_MOSI, (out >> b) & 1U);
        gpio_set_level((gpio_num_t)HOST_SCK, 0); // 下降沿：从机换位
        delayMicroseconds(2);
        gpio_set_level((gpio_num_t)HOST_SCK, 1); // 上升沿：从机采样
        delayMicroseconds(1);
        if (gpio_get_level((gpio_num_t)HOST_MISO)) {
            in |= static_cast<uint8_t>(1U << b);
        }
        delayMicroseconds(1);
    }
}

/**
 * @brief 主机侧一整帧：CS 整帧低，字节之间 20 us
 */
static void host_frame(const uint8_t *tx, int len, uint8_t *rx) {
    gpio_set_level((gpio_num_t)HOST_CS, 1);
    delayMicroseconds(50);
    gpio_set_level((gpio_num_t)HOST_CS, 0);
    delayMicroseconds(20);
    for (int i = 0; i < len; ++i) {
        host_byte(tx[i], rx[i]);
        delayMicroseconds(20);
    }
    gpio_set_level((gpio_num_t)HOST_CS, 1);
    delayMicroseconds(50);
}

/**
 * @brief SPI2 硬件从机任务：等一帧、记录、再装下一帧
 */
static void slave_task(void *arg) {
    (void)arg;

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SLV_MOSI;
    buscfg.miso_io_num = SLV_MISO;
    buscfg.sclk_io_num = SLV_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num = SLV_CS;
    slvcfg.flags = SPI_SLAVE_BIT_LSBFIRST;
    slvcfg.queue_size = 3;
    slvcfg.mode = 3;

    esp_err_t err = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    Serial.printf("spi_slave_initialize: %s\n", esp_err_to_name(err));

    // 内部回环：同一 GPIO 既要能输出（主机驱动），又要让 SPI2 的输入 matrix 看到。
    // GPIO_MODE_INPUT_OUTPUT 同时打开输入缓冲与输出驱动。
    gpio_set_direction((gpio_num_t)HOST_SCK, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction((gpio_num_t)HOST_MOSI, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction((gpio_num_t)HOST_CS, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode((gpio_num_t)HOST_MISO, GPIO_PULLUP_ONLY);

    // 固定回 0x73 模拟模式：STM32 收到后不会再发配置命令
    s_tx[0] = 0xFF; // 在线
    s_tx[1] = 0x73; // 模式字：红灯模拟，9 字节
    s_tx[2] = 0x5A; // 数据标志
    s_tx[3] = 0xFF; // 按键低字节：全松开（低有效）
    s_tx[4] = 0xFF; // 按键高字节
    s_tx[5] = 0x80; // RX
    s_tx[6] = 0x80; // RY
    s_tx[7] = 0x80; // LX
    s_tx[8] = 0x80; // LY
    for (int i = 9; i < 24; ++i) {
        s_tx[i] = 0x00;
    }

    while (true) {
        spi_slave_transaction_t t = {};
        t.length = sizeof(s_tx) * 8U;
        t.tx_buffer = s_tx;
        t.rx_buffer = s_rx;
        err = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (err == ESP_OK) {
            s_rx_len = static_cast<int>(t.trans_len / 8U);
            for (int i = 0; i < 24; ++i) {
                s_last_rx[i] = s_rx[i];
            }
            ++s_frames;
        } else {
            ++s_errors;
        }
    }
}

/* Exported functions --------------------------------------------------------*/

void setup() {
    Serial.begin(115200);
    delay(600);
    Serial.println();
    Serial.println("=== PS2 slave loopback test (ESP32-C3) ===");
    Serial.printf("host: SCK=%d MOSI=%d CS=%d MISO=%d\n", HOST_SCK, HOST_MOSI, HOST_CS, HOST_MISO);
    Serial.printf("slave: SCK=%d MOSI=%d CS=%d MISO=%d\n", SLV_SCK, SLV_MOSI, SLV_CS, SLV_MISO);

    gpio_set_direction((gpio_num_t)HOST_SCK, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction((gpio_num_t)HOST_MOSI, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction((gpio_num_t)HOST_CS, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode((gpio_num_t)HOST_MISO, GPIO_PULLUP_ONLY);
    gpio_set_level((gpio_num_t)HOST_CS, 1);
    gpio_set_level((gpio_num_t)HOST_SCK, 1);

    xTaskCreate(slave_task, "ps2_slave", 4096, nullptr, 5, nullptr);
    delay(300);
}

void loop() {
    // ① 轮询帧：0x01 0x42，期望从机回 FF 73 5A 80 80 80 80 80
    uint8_t tx[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[9] = {0};
    host_frame(tx, 9, rx);
    Serial.printf("poll  host_rx: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8]);
    Serial.printf("      slave_rx: %02X %02X %02X %02X %02X %02X %02X %02X %02X  len=%d frames=%lu err=%lu miso=%d\n",
                  s_last_rx[0], s_last_rx[1], s_last_rx[2], s_last_rx[3], s_last_rx[4],
                  s_last_rx[5], s_last_rx[6], s_last_rx[7], s_last_rx[8],
                  s_rx_len, static_cast<unsigned long>(s_frames), static_cast<unsigned long>(s_errors),
                  gpio_get_level((gpio_num_t)HOST_MISO));

    // ② 配置帧：0x01 0x43 / 0x01 0x44，验证从机能吞掉并回合法字节
    uint8_t cfg1[9] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    host_frame(cfg1, 9, rx);
    Serial.printf("cfg43 host_rx: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8]);

    uint8_t cfg2[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    host_frame(cfg2, 9, rx);
    Serial.printf("cfg44 host_rx: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8]);

    delay(1000);
}
