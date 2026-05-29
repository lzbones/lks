#include <cmath>
#include "math/angle_normalize.h"
#include "lks_config.h"

/*
 * 将角度归一化到 [-pi, pi] 区间
 *
 * @param angle 输入角度（弧度）
 * @return 归一化后的角度
 */
double normalize_angle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}