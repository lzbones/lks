#include "vehicle_model.h"

void vehicle_state_init(VehicleState* state) {
    state->x = 0.0;
    state->y = 0.0;
    state->theta = 0.0;
    state->v = 0.0;
    state->delta = 0.0;
    state->delta_prev = 0.0;
}

void vehicle_params_default(VehicleParams* params) {
    params->wheelbase = CFG_WHEELBASE;
    params->max_delta = CFG_MAX_DELTA;
    params->delta_rate = CFG_DELTA_RATE_LIMIT;
    params->max_v   = CFG_MAX_V;
    params->min_v   = CFG_MIN_V;
    params->max_a   = CFG_MAX_A;
}

void vehicle_step(VehicleState* state, const VehicleParams* params,
                  double dt) {
    /* 限制转向角幅值 */
    if (state->delta > params->max_delta)
        state->delta = params->max_delta;
    if (state->delta < -params->max_delta)
        state->delta = -params->max_delta;

    /* 欧拉积分更新位姿 */
    state->x += state->v * cos(state->theta) * dt;
    state->y += state->v * sin(state->theta) * dt;
    state->theta += (state->v / params->wheelbase) *
                    tan(state->delta) * dt;
    state->theta = normalize_angle(state->theta);

    /* 保存当前 delta 供下一时刻速率限制使用 */
    state->delta_prev = state->delta;
}

double normalize_angle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}