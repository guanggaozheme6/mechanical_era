/**
 ******************************************************************************
 * @file           : ps2_protocol.hpp
 * @brief          : PS2 报文解码（Device 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 按键位（与报文位序一致，见 PS2 解码通讯手册）
 * @note 枚举值就是报文里的位序号，1 表示按下。
 */
enum class Ps2Button : uint8_t {
    Select = 0,
    L3,
    R3,
    Start,
    Up,
    Right,
    Down,
    Left,
    L2,
    R2,
    L1,
    R1,
    Triangle,
    Circle,
    Cross,
    Square
};

/**
 * @brief 一帧解码后的手柄状态
 * @note 左右摇杆都解：左摇杆给底盘平移，右摇杆给机械臂末端速度（v0.4 起）。
 */
struct Ps2Frame {
    bool connected = false;  // 三重校验全部通过才算在线
    bool analog = false;     // 0x73 / 0x79 为模拟模式，0x41 为绿灯数字模式
    uint16_t buttons = 0;    // 1 表示按下；仅在解码入口转换协议的低有效位
    uint8_t left_x = 128;    // 左摇杆 X，0..255，128 为中位
    uint8_t left_y = 128;    // 左摇杆 Y，0..255，0x00 = 推上
    uint8_t right_x = 128;   // 右摇杆 X，0..255，0x00 = 推左（机械臂前伸/回收）
    uint8_t right_y = 128;   // 右摇杆 Y，0..255，0x00 = 推上（机械臂抬升/下压）
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 返回模式字对应的完整报文长度
 * @param mode 报文第 1 字节（模式字）
 * @return 5 / 9 / 21；未知模式返回 0
 */
std::size_t ps2_packet_size(uint8_t mode);

/**
 * @brief 解码一帧报文
 * @details 三重校验：bytes[0] == 0xFF（手柄在线）、bytes[2] == 0x5A（数据标志）、
 *          长度与模式字对得上。任一失败返回空帧，由上层走断线逻辑。
 * @param bytes 报文缓冲
 * @param length 实际收到的字节数
 * @return 解码结果；校验失败返回默认构造（connected = false）的空帧
 */
Ps2Frame decode_ps2_frame(const uint8_t *bytes, std::size_t length);

/**
 * @brief 查询某个按键是否按下
 * @param frame 解码后的帧
 * @param button 按键
 * @return true 表示按下；断线帧恒为 false
 */
bool ps2_pressed(const Ps2Frame &frame, Ps2Button button);
