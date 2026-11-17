/**
 ******************************************************************************
 * @file           : pid.hpp
 * @brief          : 纯 PID 控制器（Algorithm 层）
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 一个不做任何假设的纯 PID
 * @details 不碰硬件、不读时钟、不认识电机、不认识百分比。误差、dt、增益、
 *          输出上下限全部由调用方给，换一个被控对象可以原样复用，也可以在 PC 上
 *          用假误差序列单测，不需要任何 HAL 桩。
 * @note 抗饱和：输出饱和的那一拍不采纳本次积分（与实车前内联实现逐位一致）。
 */
struct Pid {
    float kp = 0.0F;             // 比例增益
    float ki = 0.0F;             // 积分增益
    float integral_limit = 0.0F; // 积分项限幅（抗饱和）
    float output_min = 0.0F;     // 输出下限
    float output_max = 0.0F;     // 输出上限
    float integral = 0.0F;       // 积分累计量
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 配置增益与限幅，并把积分清零
 * @param pid 控制器实例
 * @param kp 比例增益
 * @param ki 积分增益
 * @param integral_limit 积分限幅
 * @param output_min 输出下限
 * @param output_max 输出上限
 */
void pid_configure(Pid &pid, float kp, float ki, float integral_limit, float output_min,
                   float output_max);

/**
 * @brief 清积分
 * @details 停车、换向、堵转后调用，避免上一次工况的积分残留。
 * @param pid 控制器实例
 */
void pid_reset(Pid &pid);

/**
 * @brief 推一拍
 * @param pid 控制器实例
 * @param error 当前误差
 * @param dt_s 采样周期（秒）
 * @return 限幅到 [output_min, output_max] 的输出
 */
float pid_update(Pid &pid, float error, float dt_s);
