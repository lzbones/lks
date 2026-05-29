#pragma once

/*
 * 角度归一化工具函数
 */

/*
 * 将角度归一化到 [-pi, pi] 区间
 *
 * @param angle 输入角度（弧度）
 * @return 归一化后的角度
 */
double normalize_angle(double angle);