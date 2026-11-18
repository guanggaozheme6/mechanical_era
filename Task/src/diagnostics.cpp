/**
 ******************************************************************************
 * @file           : diagnostics.cpp
 * @brief          : 串口状态上报实现（Task 层）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "diagnostics.hpp"
#include "hal_handles.h"
#include "motor.hpp"
#include "ps2.hpp"
#include <cstdio>

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 上报 PS2 + 底盘状态，并附带 M4 的闭环与寄存器实配
 * @details 三行：主状态（mode= 协议模式字、LX/LY/RX/RY 双摇杆）、PID4 闭环、M4 硬件配置。
 * @param frame 本帧手柄数据
 * @param command 本帧实际下发的四路电机指令
 * @param emergency_stop 板载 Key 的急停状态
 * @param key_pressed 板载 Key 当前电平（去抖后）
 */
void report_status(const Ps2Frame &frame, DriveCommand command, bool emergency_stop,
                   bool key_pressed) {
    char message[192];
    // mode= 是 PS2 协议模式字（0x41/0x73/0x79）；v0.4 起没有操作模式，op= 已删。
    const int length = std::snprintf(
        message, sizeof(message),
        "PS2 online=%lu mode=%02lX keys=%04lX LX=%lu LY=%lu RX=%lu RY=%lu M=%d,%d,%d,%d "
        "STOP=%u KEY=%u bad=%lu\r\n",
        static_cast<unsigned long>(frame.connected), static_cast<unsigned long>(ps2_debug.mode),
        static_cast<unsigned long>(frame.buttons), static_cast<unsigned long>(frame.left_x),
        static_cast<unsigned long>(frame.left_y), static_cast<unsigned long>(frame.right_x),
        static_cast<unsigned long>(frame.right_y), command.motor[0], command.motor[1],
        command.motor[2], command.motor[3], static_cast<unsigned>(emergency_stop),
        static_cast<unsigned>(key_pressed), static_cast<unsigned long>(ps2_debug.invalid_frames));
    if (length > 0 && static_cast<unsigned>(length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(length), 20);
    }

    // 输出 M4（左后轮）的闭环状态，便于对照请求的占空比。
    const int pid_length =
        std::snprintf(message, sizeof(message),
                      "PID4 target=%d ticks=%d pwm=%d stalled=%u\r\n",
                      motor_status(3).target, motor_status(3).encoder_ticks,
                      motor_status(3).output,
                      static_cast<unsigned>(motor_status(3).stalled));
    if (pid_length > 0 && static_cast<unsigned>(pid_length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(pid_length), 20);
    }

    // TIM1/GPIO 的实配寄存器值；GPIO 配置字段：PA11=0xA（复用推挽输出），
    // PB10/PB11=0x2（推挽输出）。
    const int m4_hardware_length =
        std::snprintf(message, sizeof(message),
                      "M4HW-v1 req=%d ccr=%lu arr=%lu run=%u en=%u moe=%u oc=%lu "
                      "pa11=%lX pb10=%lX pb11=%lX out=%u,%u map=%08lX\r\n",
                      command.motor[3], static_cast<unsigned long>(TIM1->CCR4),
                      static_cast<unsigned long>(TIM1->ARR),
                      static_cast<unsigned>((TIM1->CR1 & TIM_CR1_CEN) != 0U),
                      static_cast<unsigned>((TIM1->CCER & TIM_CCER_CC4E) != 0U),
                      static_cast<unsigned>((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
                      static_cast<unsigned long>((TIM1->CCMR2 >> 12U) & 7U),
                      static_cast<unsigned long>((GPIOA->CRH >> 12U) & 15U),
                      static_cast<unsigned long>((GPIOB->CRH >> 8U) & 15U),
                      static_cast<unsigned long>((GPIOB->CRH >> 12U) & 15U),
                      static_cast<unsigned>((GPIOB->IDR & motor4_A_Pin) != 0U),
                      static_cast<unsigned>((GPIOB->IDR & motor4_B_Pin) != 0U),
                      static_cast<unsigned long>(AFIO->MAPR));
    if (m4_hardware_length > 0 && static_cast<unsigned>(m4_hardware_length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(m4_hardware_length), 20);
    }
}

/**
 * @brief 上报机械臂状态
 * @details S1..S4 之后追加解算反馈：xy / tgt / th / ok / lim / wall / neutral_ok / bu。
 * @param command 本帧四路舵机指令
 * @param feedback 解算与状态反馈
 */
void report_arm(const ArmCommand &command, const ArmFeedback &feedback) {
    char message[160]; // 本格式约 110 字符，留足余量
    const int length = std::snprintf(
        message, sizeof(message),
        "ARM S1=%d S2=%d S3=%d S4=%d xy=(%.1f,%.1f) tgt=(%.1f,%.1f) th=(%.1f,%.1f) "
        "ok=%u lim=%u wall=%u neutral_ok=%u bu=%u\r\n",
        command.servo[0], command.servo[1], command.servo[2], command.servo[3],
        static_cast<double>(feedback.x_mm), static_cast<double>(feedback.y_mm),
        static_cast<double>(feedback.target_x_mm), static_cast<double>(feedback.target_y_mm),
        static_cast<double>(feedback.shoulder_deg), static_cast<double>(feedback.elbow_deg),
        static_cast<unsigned>(feedback.reachable), static_cast<unsigned>(feedback.limited),
        static_cast<unsigned>(feedback.at_wall), feedback.neutral_ok ? 1U : 0U,
        static_cast<unsigned>(feedback.bringup));
    if (length > 0 && static_cast<unsigned>(length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(length), 20);
    }
}
