/**
 ******************************************************************************
 * @file           : ps2_bridge.ino
 * @brief          : PS5 DualSense -> PS2 接收器桥（ESP32 DevKit V1）
 *
 * 主机：STM32 的 PS2 主机（mechanical_era），Mode3 / LSB / 整帧 CS 低。
 * 从机：ESP32 SPI2 硬件 slave。
 * 手柄：PS5 DualSense，Bluepad32（BR/EDR）。
 *
 * 接线：
 *   STM32 PB12 (CS)   -> GPIO21
 *   STM32 PB13 (SCK)  -> GPIO18
 *   STM32 PB14 (MISO) <- GPIO19
 *   STM32 PB15 (MOSI) -> GPIO23
 *   GND               <-> GND
 ******************************************************************************
 */

#include <Bluepad32.h>
#include "driver/spi_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Private defines -----------------------------------------------------------*/

#define PIN_SCK 25
#define PIN_MISO 14
#define PIN_MOSI 27
#define PIN_CS 26
#define PIN_LED 2   // DevKit V1 板载蓝灯

/* Private variables ---------------------------------------------------------*/

static ControllerPtr s_pads[BP32_MAX_GAMEPADS];

// PS2 回复帧：FF 73 5A + 按键低/高 + RX RY LX LY
static volatile uint8_t s_tx[24] __attribute__((aligned(4))) = {0xFF, 0x73, 0x5A, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80};
static uint8_t s_rx[24] __attribute__((aligned(4)));   // SPI2 DMA 要求 4 字节对齐
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_connected = false;   // Bluepad32 连接状态
static volatile uint32_t s_last_pad_ms = 0; // 最近一次收到手柄数据的时刻
static volatile int s_dbg_dpad = 8;            // 调试：最近一次 dpad
static volatile uint16_t s_dbg_btn = 0xFFFF;   // 调试：最近一次按键位图

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 把 -511..512 的轴值压到 PS2 的 0..255（中位 128）
 */
static uint8_t map_stick(int v) {
    int x = 128 + (v * 127) / 512;
    if (x < 0) {
        x = 0;
    } else if (x > 255) {
        x = 255;
    }
    return static_cast<uint8_t>(x);
}

/**
 * @brief PS5 手柄 -> PS2 报文
 * @details PS2 按键低有效；位序见 mechanical_era Device/inc/ps2_protocol.hpp。
 */
static void map_pad(ControllerPtr ctl) {
    s_last_pad_ms = millis();
    uint16_t b = 0xFFFF; // 低有效，先全"松开"

    if (ctl->a()) b &= ~(1U << 14); // Cross
    if (ctl->b()) b &= ~(1U << 13); // Circle
    if (ctl->x()) b &= ~(1U << 15); // Square
    if (ctl->y()) b &= ~(1U << 12); // Triangle
    if (ctl->l1()) b &= ~(1U << 10);
    if (ctl->r1()) b &= ~(1U << 11);
    if (ctl->l2() || ctl->brake() > 128) b &= ~(1U << 8);
    if (ctl->r2() || ctl->throttle() > 128) b &= ~(1U << 9);
    if (ctl->thumbL()) b &= ~(1U << 1);
    if (ctl->thumbR()) b &= ~(1U << 2);
    if (ctl->miscStart()) b &= ~(1U << 3);
    if (ctl->miscSelect()) b &= ~(1U << 0);

    // Bluepad32 的 dpad 是位图：BIT(0)=Up, BIT(1)=Down, BIT(2)=Right, BIT(3)=Left。
    // 不是 0..8 的方向索引，必须按位取。
    const uint8_t d = ctl->dpad();
    if (d & 0x01) b &= ~(1U << 4); // Up
    if (d & 0x04) b &= ~(1U << 5); // Right
    if (d & 0x02) b &= ~(1U << 6); // Down
    if (d & 0x08) b &= ~(1U << 7); // Left

    portENTER_CRITICAL(&s_mux);
    s_dbg_btn = b;
    s_dbg_dpad = ctl->dpad();
    s_tx[3] = b & 0xFF;
    s_tx[4] = b >> 8;
    s_tx[7] = map_stick(ctl->axisX());
    s_tx[8] = map_stick(ctl->axisY());
    s_tx[5] = map_stick(ctl->axisRX());
    s_tx[6] = map_stick(ctl->axisRY());
    portEXIT_CRITICAL(&s_mux);
}

/**
 * @brief SPI2 硬件从机：等主机 CS 拉低，整帧交换
 */
static void ps2_task(void *arg) {
    (void)arg;

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = PIN_MISO;
    buscfg.sclk_io_num = PIN_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.spics_io_num = PIN_CS;
    slvcfg.flags = SPI_SLAVE_BIT_LSBFIRST;
    slvcfg.queue_size = 3;
    slvcfg.mode = 3;

    spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

    while (true) {
        spi_slave_transaction_t t = {};
        t.length = sizeof(s_tx) * 8U;
        t.tx_buffer = (void *)s_tx;
        t.rx_buffer = s_rx;
        spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
    }
}

/* Exported functions --------------------------------------------------------*/

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (s_pads[i] == nullptr) {
            s_pads[i] = ctl;
            s_connected = true;
            s_last_pad_ms = millis();
            break;
        }
    }
    Serial.printf("connected: %s\n", ctl->getModelName().c_str());
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (s_pads[i] == ctl) {
            s_pads[i] = nullptr;
        }
    }
    s_connected = false;
    Serial.println("disconnected");
}

void setup() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    Serial.begin(921600);
    delay(300);
    Serial.println("\nPS2 bridge: PS5 -> PS2");

    BP32.setup(&onConnectedController, &onDisconnectedController);
    xTaskCreatePinnedToCore(ps2_task, "ps2_slave", 4096, nullptr, 5, nullptr, 1);
}

void loop() {
    if (BP32.update()) {
        for (auto ctl : s_pads) {
            if (ctl && ctl->isConnected() && ctl->hasData()) {
                map_pad(ctl);
            }
        }
    }
    // 断线语义：手柄不在线或 500 ms 没数据 -> 回无效帧（0x00），
    // 让 STM32 的 100 ms 有效帧超时逻辑接管，电机会停。
    const bool alive = s_connected && (millis() - s_last_pad_ms < 500);
    s_tx[0] = alive ? 0xFF : 0x00;

    // 板载灯：未连接慢闪（500 ms），连接常亮
    digitalWrite(PIN_LED, alive ? HIGH : ((millis() / 500) % 2));

    static uint32_t last_beat_us = 0;
    if (micros() - last_beat_us > 3333) {   // 300 Hz 心跳（3.33 ms），串口 921600
        last_beat_us = micros();
        char beat[96];
        snprintf(beat, sizeof(beat), "[beat] conn=%d tx0=0x%02X dpad=%d btn=%04X lx=%03d ly=%03d rx=%03d ry=%03d", (int)s_connected, s_tx[0], s_dbg_dpad, s_dbg_btn, s_tx[7], s_tx[8], s_tx[5], s_tx[6]);
        Serial.println(beat);
    }
    delay(1);
}
