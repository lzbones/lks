#pragma once

/*
 * 车辆状态与参数结构体定义 + 函数声明
 */

#include "lks_config.h"
#include <cmath>

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
} vehicle_state_t;

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
} vehicle_params_t;

/* ---- 函数声明 ---- */

/*
 * 初始化车辆状态为零值
 */
void vehicle_state_init(vehicle_state_t* state);

/*
 * 使用默认参数初始化车辆参数
 */
void vehicle_params_default(vehicle_params_t* params);

/*
 * 限制转向角变化速率: |delta - delta_prev| <= delta_rate_limit
 *
 * @param state           指向车辆状态的指针
 * @param delta_rate_limit 允许的转角变化上限 (rad), = params->delta_rate * dt
 */
void vehicle_delta_rate_limit(vehicle_state_t* state,
                              double delta_rate_limit);

/*
 * 自行车模型一步前向积分 (欧拉法, dt 秒)
 *   dx/dt = v * cos(theta)
 *   dy/dt = v * sin(theta)
 *   dtheta/dt = (v / L) * tan(delta)
 *
 * @param state 指向车辆状态的指针
 * @param params 指向车辆参数的指针
 * @param dt 积分步长（秒）
 */
void vehicle_step(vehicle_state_t* state,
                  const vehicle_params_t* params,
                  double dt);