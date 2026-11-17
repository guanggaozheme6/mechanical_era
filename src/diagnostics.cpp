#include "diagnostics.hpp"
#include "motor.hpp"
#include "ps2.hpp"
#include <cstdio>

extern "C" UART_HandleTypeDef huart3;

void report_status(const Ps2Frame &frame, DriveCommand command, bool emergency_stop,
                   bool key_pressed, bool arm_mode) {
    char message[192];
    // mode= 是 PS2 协议模式字（0x41/0x73/0x79）；操作模式另起 op=，别混用。
    const int length = std::snprintf(
        message, sizeof(message),
        "PS2 online=%lu mode=%02lX op=%s keys=%04lX LX=%lu LY=%lu M=%d,%d,%d,%d STOP=%u KEY=%u "
        "bad=%lu\r\n",
        static_cast<unsigned long>(frame.connected), static_cast<unsigned long>(ps2_debug.mode),
        arm_mode ? "ARM" : "DRV", static_cast<unsigned long>(frame.buttons),
        static_cast<unsigned long>(frame.left_x), static_cast<unsigned long>(frame.left_y),
        command.motor[0], command.motor[1], command.motor[2], command.motor[3],
        static_cast<unsigned>(emergency_stop), static_cast<unsigned>(key_pressed),
        static_cast<unsigned long>(ps2_debug.invalid_frames));
    if (length > 0 && static_cast<unsigned>(length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(length), 20);
    }

    // 读取 M4 的实际硬件配置，便于对照请求的占空比。
    // GPIO 配置字段：PA11=0xA（复用推挽输出），PB10/PB11=0x2
    // （推挽输出）
    const int hardware_length =
        // M4 是左后轮；输出其闭环状态，排查单轮编码器或 PID 参数问题。
        std::snprintf(message, sizeof(message),
                      "PID4 target=%d ticks=%d pwm=%d stalled=%u\r\n",
                      motor_status(3).target, motor_status(3).encoder_ticks,
                      motor_status(3).output,
                      static_cast<unsigned>(motor_status(3).stalled));
    if (hardware_length > 0 && static_cast<unsigned>(hardware_length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(hardware_length), 20);
    }

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

void report_arm(const ArmCommand &command, const ArmFeedback &feedback) {
    // 新格式约 105 字符，128 已无余量；缓冲按 arm_pose_v0.3 §8 加到 160。
    char message[160];
    const int length = std::snprintf(
        message, sizeof(message),
        "ARM S1=%d S2=%d S3=%d S4=%d xy=(%.1f,%.1f) tgt=(%.1f,%.1f) th=(%.1f,%.1f) "
        "ok=%u lim=%u wall=%u poses=0x%02X bu=%u\r\n",
        command.servo[0], command.servo[1], command.servo[2], command.servo[3],
        static_cast<double>(feedback.x_mm), static_cast<double>(feedback.y_mm),
        static_cast<double>(feedback.target_x_mm), static_cast<double>(feedback.target_y_mm),
        static_cast<double>(feedback.shoulder_deg), static_cast<double>(feedback.elbow_deg),
        static_cast<unsigned>(feedback.reachable), static_cast<unsigned>(feedback.limited),
        static_cast<unsigned>(feedback.at_wall), static_cast<unsigned>(feedback.bad_poses),
        static_cast<unsigned>(feedback.bringup));
    if (length > 0 && static_cast<unsigned>(length) < sizeof(message)) {
        HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t *>(message),
                          static_cast<uint16_t>(length), 20);
    }
}
