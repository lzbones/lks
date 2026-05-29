#include "vehicle/state.h"

/*
 * 限制转向角变化速率: |delta - delta_prev| <= delta_rate_limit
 *
 * @param state           指向车辆状态的指针
 * @param delta_rate_limit 允许的转角变化上限 (rad), = params->delta_rate * dt
 */
void vehicle_delta_rate_limit(vehicle_state_t* state,
                              double delta_rate_limit) {
    const double diff = state->delta - state->delta_prev;

    if (diff > delta_rate_limit) {
        state->delta = state->delta_prev + delta_rate_limit;
    } else if (diff < -delta_rate_limit) {
        state->delta = state->delta_prev - delta_rate_limit;
    }
}