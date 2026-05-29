#include "vehicle/state.h"

/*
 * 使用默认参数初始化车辆参数
 *
 * @param params 指向车辆参数的指针
 */
void vehicle_params_default(vehicle_params_t* params) {
    params->wheelbase = LKS_WHEELBASE;
    params->max_delta = LKS_MAX_DELTA;
    params->delta_rate = LKS_DELTA_RATE_LIMIT;
    params->max_v = LKS_MAX_V;
    params->min_v = LKS_MIN_V;
    params->max_a = LKS_MAX_A;
}