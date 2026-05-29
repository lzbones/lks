#include "simulation/speed_pid.h"

/*
 * 简单的速度跟踪: P 控制器调节加速度
 *
 * @param current_v   当前速度 (m/s)
 * @param target_v    目标速度 (m/s)
 * @param max_a       最大加速度幅值 (m/s²)
 * @return 计算出的加速度 (m/s²)
 */
double simulation_speed_pid(double current_v,
                            double target_v,
                            double max_a) {
    const double a_unclamped = (target_v - current_v) * SPEED_P_GAIN;
    const double lo = -max_a;
    const double hi = max_a;
    return (a_unclamped < lo) ? lo : ((a_unclamped > hi) ? hi : a_unclamped);
}