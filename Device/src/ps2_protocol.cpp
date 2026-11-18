/**
 ******************************************************************************
 * @file           : ps2_protocol.cpp
 * @brief          : PS2 报文解码实现（Device 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ps2_protocol.hpp"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 返回模式字对应的完整报文长度
 * @param mode 报文第 1 字节（模式字）
 * @return 5（0x41）/ 9（0x73）/ 21（0x79）；未知模式返回 0
 */
std::size_t ps2_packet_size(uint8_t mode) {
    switch (mode) {
    case 0x41: // 绿灯模式：无模拟量
        return 5;
    case 0x73: // 红灯模式：模拟量
        return 9;
    case 0x79: // 双震动模拟模式
        return 21;
    default:
        return 0;
    }
}

/**
 * @brief 解码一帧报文（三重校验）
 * @details bytes[0]==0xFF（在线）、bytes[2]==0x5A（数据标志）、长度与模式字对得上。
 * @param bytes 报文缓冲
 * @param length 实际收到的字节数
 * @return 解码结果；校验失败返回 connected=false 的空帧
 */
Ps2Frame decode_ps2_frame(const uint8_t *bytes, std::size_t length) {
    if (bytes == nullptr || length < 3) {
        return {};
    }
    const auto expected = ps2_packet_size(bytes[1]);
    if (expected == 0 || length < expected || bytes[0] != 0xFF || bytes[2] != 0x5A) {
        return {};
    }
    Ps2Frame frame;
    frame.connected = true;
    frame.analog = bytes[1] != 0x41;
    frame.buttons = static_cast<uint16_t>(
        ~(static_cast<uint16_t>(bytes[3]) | (static_cast<uint16_t>(bytes[4]) << 8)));
    if (frame.analog) {
        // 字节 5/6 是右摇杆，本工程不用，直接跳过。
        frame.left_x = bytes[7];
        frame.left_y = bytes[8];
    }
    return frame;
}

/**
 * @brief 查询某个按键是否按下
 * @param frame 解码后的帧
 * @param button 按键
 * @return true 表示按下；断线帧恒为 false
 */
bool ps2_pressed(const Ps2Frame &frame, Ps2Button button) {
    const auto bit = static_cast<unsigned>(button);
    return frame.connected && bit < 16 && (frame.buttons & (1U << bit)) != 0;
}
