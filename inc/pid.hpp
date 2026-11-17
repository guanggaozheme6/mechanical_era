#pragma once

// ============================================================================
//  pid —— Algorithm 层：一个不做任何假设的纯 PID
// ----------------------------------------------------------------------------
//  不碰硬件、不读时钟、不认识电机、不认识百分比。
//  误差、dt、增益、输出上下限全部由调用方给 —— 换一个被控对象可以原样复用，
//  也可以在 PC 上用假误差序列单测，不需要任何 HAL 桩。
//  抗饱和：输出饱和的那一拍不采纳本次积分（与实车前一致）。
// ============================================================================

struct Pid {
    float kp = 0.0F;
    float ki = 0.0F;
    float integral_limit = 0.0F;
    float output_min = 0.0F;
    float output_max = 0.0F;
    float integral = 0.0F;
};

void pid_configure(Pid &pid, float kp, float ki, float integral_limit, float output_min,
                   float output_max);

// 清积分（停车、换向、堵转后调用）。
void pid_reset(Pid &pid);

// 推一拍：输入误差，返回 [output_min, output_max] 内的输出。
float pid_update(Pid &pid, float error, float dt_s);
