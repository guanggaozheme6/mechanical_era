# ESP32 侧：PS5 手柄 → PS2 接收端替代

目标：用一块 ESP32 冒充 PS2 接收器，把 **PS5 DualSense** 的输入按 PS2 报文格式回给 STM32，
STM32 端零改动。

## 硬件

| 项 | 结论 |
|---|---|
| 板子 | ESP32 DevKit V1（ESP32-D0WD-V3，双核 240 MHz，4 MB flash，BR/EDR + BLE） |
| 手柄 | PS5 DualSense（标准 HID，走 BR/EDR 经典蓝牙） |
| 固件栈 | Bluepad32 v4.1.0（Arduino core `esp32-bluepad32:esp32`） |
| PS2 从机 | ESP-IDF `spi_slave`，SPI2_HOST 硬件从机 |

## 接线（接收器 6 针 → ESP32）

| 接收器 | 信号 | ESP32 |
|---|---|---|
| 1 | GND | GND（右侧，VIN 下面） |
| 2 | 3V3 | 3V3（左侧最上）※ |
| 3 | MISO | GPIO14 |
| 4 | MOSI | GPIO27 |
| 5 | NSS | GPIO26 |
| 6 | SCK | GPIO25 |

※ 建议 ESP32 用 USB 供电、只与接收器共地。接收器 3.3V 的电流余量不足以覆盖蓝牙峰值，
接上去可能反复复位。

SPI 四根在右侧连续（14→27→26→25）；GPIO12 是 strapping 脚，特意跳过。

## 报文

ESP 回的 9 字节（固定 0x73 模拟模式）：

```
0xFF 0x73 0x5A btn_lo btn_hi RX RY LX LY
```

- 按键低有效：`buttons = ~(b3 | b4 << 8)`，位序见 STM32 `Device/inc/ps2_protocol.hpp`
- 摇杆 0..255，中位 128；字节 5/6 是右摇杆（机械臂），7/8 是左摇杆（底盘）
- SPI：Mode 3、LSB first、CS 整帧低，与 `Driver/src/ps2.cpp` 的主机时序一致

## 按键映射

| PS5 | PS2 位 |
|---|---|
| × Cross | bit14 |
| ○ Circle | bit13 |
| □ Square | bit15 |
| △ Triangle | bit12 |
| L1 / R1 | bit10 / bit11 |
| L2 / R2 | bit8 / bit9（含扳机 >128 阈值） |
| L3 / R3 | bit1 / bit2 |
| Options | bit3 Start |
| Create | bit0 Select |
| D-pad | bit4-7 |

## 断线语义

- 手柄断线，或 500 ms 没有新数据 → 回无效帧（`s_tx[0] = 0x00`）
- STM32 的 `decode_ps2_frame` 校验失败 → 100 ms 后 `ps2_read()` 返回空帧 → 电机停、机械臂冻结
- 这条是为安全加的：原版接收器断线时直接不回帧，ESP 必须主动模拟"不在线"

## 编译烧录

```powershell
arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32 ps2_bridge
arduino-cli upload  --fqbn esp32-bluepad32:esp32:esp32 -p COM10 ps2_bridge
```

烧录必须插 USB；烧完可以继续用 USB 供电，只接 GND 到接收器。

## 已知差异（与原件接收器）

| 项 | 原件 | ESP |
|---|---|---|
| 模式字 | 0x41 / 0x73 / 0x79 | 固定 0x73 |
| 配置命令回复 | 0x43→0xF3、0x44→0x5A 等 | 不解析，固定回数据帧 |
| 断线 | 不回帧 | 回无效帧（0x00） |
| 帧内原子性 | 硬件锁存 | 可能撕裂（低概率） |

STM32 的 `configure_analog()` 只看 HAL 状态、不校验回复，而且固定 0x73 让它永远不会发配置
命令，所以前两项对主机无影响。

## 目录

```
esp32/
├── README.md
├── ps2_bridge/           # 当前方案：Bluepad32 + PS2 从机（DevKit V1）
├── legacy/               # 历史验证，仅备查
│   └── ps2_slave_test/   # C3 自环：软件主机 + SPI2 从机，验证 PS2 协议
└── tools/
    ├── padmon2.ps1/.bat  # COM10 Bluepad32 串口面板
    └── ps5_mon.py/.bat   # Windows USB 直连 PS5 的探测面板
```

## 历史

- 最初路线是 ESP32-C3 + 墨将 Rainbow 2 Pro：C3 只有 BLE、没有 BR/EDR，墨将走私有协议
  （`FFE1/FFE2` 配置通道 + D2 报告，只有按键没有摇杆），放弃。
- 换 DevKit V1（有 BR/EDR）+ PS5 DualSense（标准 HID），Bluepad32 原生支持 DualSense。
- PS2 从机协议最早在 C3 上自环验证通过（`legacy/ps2_slave_test`），引脚不同、逻辑一致。
- 摇杆方向尚未实车确认：`map_stick()` 假设 Bluepad32 的 `axisY()` 正方向为下（推下→0xFF）。
