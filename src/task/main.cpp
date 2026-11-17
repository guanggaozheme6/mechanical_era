#include "app.h"
#include "task/diagnostics.hpp"
#include "task/mode_control.hpp"
#include "driver/outputs.hpp"
#include "driver/ps2.hpp"

extern "C" SPI_HandleTypeDef hspi2;

extern "C" void main_task() {
    init_outputs();
    ps2_init(hspi2);
    mode_control_init(HAL_GetTick());

    constexpr uint32_t kReportMs = 500;
    uint32_t last_report = HAL_GetTick() - kReportMs;
    bool emergency_stop = false;
    bool key_sample = false;
    bool key_pressed = false;
    uint32_t key_change_ms = HAL_GetTick();

    while (true) {
        // 板载 Key：20 ms 去抖后按一次锁定急停，再按一次解除。
        // 手柄断线时 mode_control 也会自行停止，这条是硬件层面的第二道保险。
        const bool key_now = HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET;
        if (key_now != key_sample) {
            key_sample = key_now;
            key_change_ms = HAL_GetTick();
        } else if (key_now != key_pressed && HAL_GetTick() - key_change_ms >= 20U) {
            key_pressed = key_now;
            if (key_pressed) {
                emergency_stop = !emergency_stop;
            }
        }

        ps2_update();
        const Ps2Frame frame = ps2_read();
        const ControlOutput output = mode_control_update(frame, HAL_GetTick());

        const bool stopped = output.stopped || emergency_stop;
        const DriveCommand drive = stopped ? DriveCommand{} : output.drive;
        apply_motors(drive);
        // 急停时机械臂保持当前姿态（mode_control 已处理），不会甩回中位。
        apply_servos(output.arm);

        // 标定时臂是静止的（松手即停），500 ms 一次够抄数。
        const uint32_t report_ms = kReportMs;
        if (HAL_GetTick() - last_report >= report_ms) {
            report_status(frame, drive, stopped, key_pressed,
                          output.mode == OperationMode::Arm);
            report_arm(output.arm, output.arm_fb);
            last_report = HAL_GetTick();
        }
        HAL_Delay(1);
    }
}
