#include <cmath>
#include "vehicle/state.h"
#include "math/angle_normalize.h"

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
                  double dt) {
    /* 限制转向角幅值 */
    const double delta_clamped = (state->delta > params->max_delta)
        ? params->max_delta
        : ((state->delta < -params->max_delta)
            ? -params->max_delta
            : state->delta);

    /* 欧拉积分更新位姿 */
    const double dx = state->v * cos(state->theta) * dt;
    const double dy = state->v * sin(state->theta) * dt;
    const double dtheta = (state->v / params->wheelbase) *
                          tan(delta_clamped) * dt;

    state->x += dx;
    state->y += dy;
    state->theta = normalize_angle(state->theta + dtheta);

    /* 更新 delta 为限制后的值并保存供下一时刻速率限制使用 */
    state->delta = delta_clamped;
    state->delta_prev = delta_clamped;
}