#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include <cmath>
#include "config.h"

/*
 * 车辆状态结构体
 */
typedef struct {
    double x;           /* 横坐标 (m) */
    double y;           /* 纵坐标 (m) */
    double theta;       /* 航向角 (rad) */
    double v;           /* 速度 (m/s) */
    double delta;       /* 当前前轮转角 (rad) */
    double delta_prev;  /* 上一时刻前轮转角 (rad), 用于速率限制 */
} VehicleState;

/*
 * 车辆参数结构体
 */
typedef struct {
    double wheelbase;   /* 轴距 (m) */
    double max_delta;   /* 最大转向角 (rad) */
    double delta_rate;  /* 转向速率上限 (rad/s) */
    double max_v;       /* 最大速度 (m/s) */
    double min_v;       /* 最小速度 (m/s) */
    double max_a;       /* 最大加速度 (m/s²) */
} VehicleParams;

/*
 * 初始化车辆状态为零
 */
void vehicle_state_init(VehicleState* state);

/*
 * 使用默认参数初始化车辆参数
 */
void vehicle_params_default(VehicleParams* params);

/*
 * 限制转向角变化速率: |delta - delta_prev| <= delta_rate_limit * dt
 * delta_rate_limit: 允许的转角变化上限 (rad), = params->delta_rate * dt
 * 调用方需在调用前保存上一时刻的 delta 到 state->delta_prev
 */
static inline void vehicle_limit_delta_rate(VehicleState* state,
                                            double delta_rate_limit) {
    double diff = state->delta - state->delta_prev;
    if (diff > delta_rate_limit) {
        state->delta = state->delta_prev + delta_rate_limit;
    } else if (diff < -delta_rate_limit) {
        state->delta = state->delta_prev - delta_rate_limit;
    }
}

/*
 * 自行车模型一步前向积分 (欧拉法, dt 秒)
 *   dx/dt = v * cos(theta)
 *   dy/dt = v * sin(theta)
 *   dtheta/dt = (v / L) * tan(delta)
 */
void vehicle_step(VehicleState* state, const VehicleParams* params,
                  double dt);

/*
 * 将角度归一化到 [-pi, pi]
 */
double normalize_angle(double angle);

#endif /* VEHICLE_MODEL_H */