#include "vehicle/state.h"

/*
 * 初始化车辆状态为零值
 *
 * @param state 指向车辆状态的指针
 */
void vehicle_state_init(vehicle_state_t* state) {
    state->x = 0.0;
    state->y = 0.0;
    state->theta = 0.0;
    state->v = 0.0;
    state->delta = 0.0;
    state->delta_prev = 0.0;
}