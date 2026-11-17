#pragma once

#include "arm_control.hpp"
#include "arm_pose.hpp"
#include "drive_control.hpp"

// arm_mode 只用于串口打 op=DRV/ARM（操作模式）。mode= 那个字段是 PS2 协议模式字，
// 两者字段名必须分开，否则现场一定读错。
void report_status(const Ps2Frame &frame, DriveCommand command, bool emergency_stop,
                   bool key_pressed, bool arm_mode);

// 机械臂四路姿态 + 解算反馈。标定零点 / 度每百分比 / 预设表时抄这条输出。
void report_arm(const ArmCommand &command, const ArmFeedback &feedback);
