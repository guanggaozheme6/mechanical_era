/**
 ******************************************************************************
 * @file           : keymap_test.cpp
 * @brief          : v0.4 按键映射的主机侧验收（真代码，不烧板）
 ****************************************************************************** *
 * 用法（在仓库根目录，一行）：
 *   g++ -std=c++14 -O2 -I Include -I Algorithm/inc -I Device/inc -I Task/inc sim/verify/keymap_test.cpp Task/src/mode_control.cpp Device/src/arm_pose.cpp Device/src/arm_joint.cpp Device/src/drive_control.cpp Device/src/arm_control.cpp Device/src/ps2_protocol.cpp Algorithm/src/arm_kinematics.cpp Algorithm/src/arm_limit.cpp -o sim/verify/keymap_test.exe && ./sim/verify/keymap_test.exe
 *
 * 验什么：v0.4 只改「输入怎么接」，所以这里按手柄帧逐条走一遍真链路
 * （ps2_protocol → mode_control → drive_control / arm_pose / arm_limit / arm_joint）：
 *   K1 右摇杆字节真的进了 Ps2Frame（bytes 5/6）
 *   K2 左摇杆只动底盘，不动机械臂
 *   K3 右摇杆只动机械臂（末端 x 增加），底盘四路全 0
 *   K4 L1/R1 原地自转（左右轮反向），L2/R2 不再参与底盘
 *   K5 L2/R2 = 夹爪 松/紧（S3 变化），不影响 S1/S2
 *   K6 ←/→ = 腕 roll（正 = 左转），不影响 S3
 *   K7 Start = 回中立位（fb.target 回到 kNeutralPose），且不动 S3/S4
 *   K8 旧预设键（↑↓△○╳□）已无功能：什么都不动
 *   K9 arm_pose_request_xy() 仍可用（上位机 / 动作序列的唯一入口，v0.4 没有按键调它）
 */
#include "mode_control.hpp"
#include "ps2_protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>

namespace {

int g_failures = 0;

/**
 * @brief 断言
 * @param ok 条件
 * @param name 用例名
 * @param detail 说明
 */
void check(bool ok, const char *name, const char *detail) {
    if (!ok) {
        ++g_failures;
    }
    std::printf("%s  %-46s %s\n", ok ? "  PASS  " : "  FAIL  ", name, detail);
}

/** @brief 造一帧手柄数据（模拟模式，中位 128） */
Ps2Frame frame_with(uint16_t buttons) {
    Ps2Frame frame;
    frame.connected = true;
    frame.analog = true;
    frame.buttons = buttons;
    return frame;
}

/** @brief 按位给出按键掩码 */
uint16_t bit(Ps2Button button) { return static_cast<uint16_t>(1U << static_cast<unsigned>(button)); }

/** @brief 四路电机是否全零 */
bool drive_zero(const DriveCommand &command) {
    for (int m : command.motor) {
        if (m != 0) {
            return false;
        }
    }
    return true;
}

/** @brief 一份「当前指令」快照，用来判断某个键有没有动过机械臂 */
struct ArmSnap {
    int s1;
    int s2;
    int s3;
    int s4;
    float tx;
    float ty;
};

ArmSnap snap(const ControlOutput &output) {
    ArmSnap s{output.arm.servo[0], output.arm.servo[1], output.arm.servo[2], output.arm.servo[3],
              output.arm_fb.target_x_mm, output.arm_fb.target_y_mm};
    return s;
}

/**
 * @brief 跑若干帧，返回最后一帧的输出
 * @param frames 帧数
 * @param make 每帧造数据（收到帧号）
 * @param start_ms 起始时基
 * @param step_ms 每帧毫秒
 */
template <typename Fn>
ControlOutput run(unsigned frames, Fn make, uint32_t start_ms, uint32_t step_ms) {
    ControlOutput last;
    for (unsigned i = 0; i < frames; ++i) {
        last = mode_control_update(make(i), start_ms + i * step_ms);
    }
    return last;
}

} // namespace

int main() {
    /* ---- K1：报文解码把右摇杆带进来 ---- */
    {
        uint8_t packet[9] = {0xFF, 0x73, 0x5A, 0xFF, 0xFF, 200, 30, 12, 240};
        const Ps2Frame frame = decode_ps2_frame(packet, sizeof(packet));
        char detail[96];
        std::snprintf(detail, sizeof(detail), "right=(%u,%u) left=(%u,%u)",
                      static_cast<unsigned>(frame.right_x), static_cast<unsigned>(frame.right_y),
                      static_cast<unsigned>(frame.left_x), static_cast<unsigned>(frame.left_y));
        check(frame.connected && frame.right_x == 200 && frame.right_y == 30 && frame.left_x == 12 &&
                  frame.left_y == 240,
              "K1 右摇杆字节（5/6）进 Ps2Frame", detail);
    }

    mode_control_init(0);

    /* ---- K2：左摇杆只动底盘 ---- */
    {
        const ArmSnap before = snap(mode_control_update(frame_with(0), 0));
        ControlOutput last;
        for (unsigned i = 1; i <= 20; ++i) {
            Ps2Frame frame = frame_with(0);
            frame.left_y = 0;   // 前进到底（0x00 = 推上）
            frame.right_x = 128;
            frame.right_y = 128;
            last = mode_control_update(frame, i * 20U);
        }
        const ArmSnap after = snap(last);
        char detail[128];
        std::snprintf(detail, sizeof(detail), "M=(%d,%d,%d,%d) 机械臂 S1=%d→%d S2=%d→%d",
                      last.drive.motor[0], last.drive.motor[1], last.drive.motor[2],
                      last.drive.motor[3], before.s1, after.s1, before.s2, after.s2);
        check(last.drive.motor[0] > 0 && last.drive.motor[1] > 0 && last.drive.motor[2] > 0 &&
                  last.drive.motor[3] > 0 && before.s1 == after.s1 && before.s2 == after.s2,
              "K2 左摇杆只动底盘（不碰机械臂）", detail);
    }

    /* ---- K3：右摇杆只动机械臂 ---- */
    {
        const ArmSnap before = snap(mode_control_update(frame_with(0), 1000));
        ControlOutput last;
        for (unsigned i = 1; i <= 40; ++i) {
            Ps2Frame frame = frame_with(0);
            frame.left_x = 128;
            frame.left_y = 128;
            frame.right_x = 255; // 推到最右 = 前伸
            frame.right_y = 128;
            last = mode_control_update(frame, 1000U + i * 20U);
        }
        const ArmSnap after = snap(last);
        char detail[160];
        std::snprintf(detail, sizeof(detail), "末端 x %.1f→%.1f（期望增大），底盘 M=(%d,%d,%d,%d)",
                      static_cast<double>(before.tx), static_cast<double>(after.tx),
                      last.drive.motor[0], last.drive.motor[1], last.drive.motor[2],
                      last.drive.motor[3]);
        check(after.tx > before.tx + 1.0F && drive_zero(last.drive),
              "K3 右摇杆只动机械臂（底盘全 0）", detail);
    }

    /* ---- K4：L1/R1 自转，L2/R2 不参与底盘 ---- */
    {
        Ps2Frame frame = frame_with(bit(Ps2Button::R1));
        const DriveCommand cw = drive_from_ps2(frame);
        frame = frame_with(bit(Ps2Button::L1));
        const DriveCommand ccw = drive_from_ps2(frame);
        frame = frame_with(bit(Ps2Button::R2));
        const DriveCommand trig = drive_from_ps2(frame);
        char detail[160];
        std::snprintf(detail, sizeof(detail), "R1=(%d,%d,%d,%d) L1=(%d,%d,%d,%d) R2 全零=%d",
                      cw.motor[0], cw.motor[1], cw.motor[2], cw.motor[3], ccw.motor[0], ccw.motor[1],
                      ccw.motor[2], ccw.motor[3], drive_zero(trig) ? 1 : 0);
        const bool split = cw.motor[0] != 0 && cw.motor[1] != 0 &&
                           ((cw.motor[0] > 0) != (cw.motor[1] > 0));
        check(split && ccw.motor[0] == -cw.motor[0] && drive_zero(trig),
              "K4 L1/R1 原地自转，扳机不再转底盘", detail);
    }

    /* ---- K5：扳机 = 夹爪（S3），不动 S1/S2 ---- */
    {
        const ArmSnap before = snap(mode_control_update(frame_with(0), 2000));
        ControlOutput last;
        for (unsigned i = 1; i <= 20; ++i) {
            last = mode_control_update(frame_with(bit(Ps2Button::L2)), 2000U + i * 20U);
        }
        const ArmSnap after = snap(last);
        char detail[160];
        std::snprintf(detail, sizeof(detail), "S3 %d→%d（L2 松 = 正行程），S1 %d→%d S2 %d→%d",
                      before.s3, after.s3, before.s1, after.s1, before.s2, after.s2);
        check(after.s3 > before.s3 && after.s1 == before.s1 && after.s2 == before.s2,
              "K5 L2/R2 只动夹爪 S3", detail);
    }

    /* ---- K6：方向键左右 = 腕 roll，正 = 左转 ---- */
    {
        const ArmSnap before = snap(mode_control_update(frame_with(0), 3000));
        ControlOutput left_out;
        for (unsigned i = 1; i <= 20; ++i) {
            left_out = mode_control_update(frame_with(bit(Ps2Button::Left)), 3000U + i * 20U);
        }
        const ArmSnap left = snap(left_out);
        ControlOutput right_out;
        for (unsigned i = 1; i <= 40; ++i) {
            right_out = mode_control_update(frame_with(bit(Ps2Button::Right)), 3400U + i * 20U);
        }
        const ArmSnap right = snap(right_out);
        char detail[160];
        std::snprintf(detail, sizeof(detail), "← S4 %d→%d，→ S4 →%d（正=左转），S3 %d→%d",
                      before.s4, left.s4, right.s4, before.s3, left.s3);
        check(left.s4 > before.s4 && right.s4 < left.s4 && left.s3 == before.s3,
              "K6 ←/→ 只动腕 S4（正 = 左转）", detail);
    }

    /* ---- K7：Start 回中立位，不动 S3/S4 ---- */
    {
        const ArmSnap before = snap(mode_control_update(frame_with(0), 5000));
        const ControlOutput after = mode_control_update(frame_with(bit(Ps2Button::Start)), 5020U);
        const ArmSnap s = snap(after);
        char detail[192];
        std::snprintf(detail, sizeof(detail),
                      "tgt %.1f→(%.1f,%.1f) 期望 (105.2,38.3)；S3 %d→%d S4 %d→%d",
                      static_cast<double>(before.tx), static_cast<double>(s.tx),
                      static_cast<double>(s.ty), before.s3, s.s3, before.s4, s.s4);
        check(std::fabs(s.tx - 105.2F) < 0.4F && std::fabs(s.ty - 38.3F) < 0.4F &&
                  s.s3 == before.s3 && s.s4 == before.s4,
              "K7 Start 回中立位且不碰夹爪/腕", detail);
    }

    /* ---- K8：旧预设键已无功能 ---- */
    {
        const ArmSnap before = snap(mode_control_update(frame_with(0), 6000));
        const uint16_t legacy = static_cast<uint16_t>(bit(Ps2Button::Up) | bit(Ps2Button::Down) |
                                                      bit(Ps2Button::Triangle) |
                                                      bit(Ps2Button::Circle) |
                                                      bit(Ps2Button::Cross) |
                                                      bit(Ps2Button::Square) |
                                                      bit(Ps2Button::Select));
        const ArmSnap after = snap(mode_control_update(frame_with(legacy), 6020U));
        char detail[160];
        std::snprintf(detail, sizeof(detail), "tgt=(%.1f,%.1f)（按前 %.1f,%.1f）S1..S4 未变",
                      static_cast<double>(after.tx), static_cast<double>(after.ty),
                      static_cast<double>(before.tx), static_cast<double>(before.ty));
        check(after.tx == before.tx && after.ty == before.ty && after.s1 == before.s1 &&
                  after.s2 == before.s2 && after.s3 == before.s3 && after.s4 == before.s4,
              "K8 旧预设键 / Select 已无功能", detail);
    }

    /* ---- K9：绝对坐标入口（上位机 / 动作序列）仍可用 ---- */
    {
        const bool ok = arm_pose_request_xy(114.0F, 0.0F);
        const ArmFeedback feedback = arm_pose_feedback();
        char detail[160];
        std::snprintf(detail, sizeof(detail), "request_xy(114,0) → tgt=(%.1f,%.1f) 命中=%d",
                      static_cast<double>(feedback.target_x_mm),
                      static_cast<double>(feedback.target_y_mm), ok ? 1 : 0);
        check(ok && std::fabs(feedback.target_x_mm - 114.0F) < 0.4F &&
                  std::fabs(feedback.target_y_mm) < 0.4F,
              "K9 arm_pose_request_xy 直接给坐标", detail);
    }

    std::printf("\n%s（%d 项失败）\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
