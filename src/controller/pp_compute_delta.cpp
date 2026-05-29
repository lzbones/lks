#include "controller/pure_pursuit.h"
#include "math/angle_normalize.h"
#include <math.h>

/*
 * 纯跟踪控制: 计算转向角 delta.
 *
 * 算法:
 *   1. 根据当前速度计算前瞻距离 L = max(min_lookahead, gain × v)
 *   2. 在车道中心线上找到距车辆约 L 米的前视点 (gx, gy)
 *   3. delta = 2 × L_c × sin(theta_c) / wheelbase
 *      其中 theta_c 为朝向矢量与车辆航向的夹角
 *
 * @param state    指向车辆状态的指针
 * @param vparams  指向车辆参数的指针
 * @param lane     指向车道模型的指针
 * @param cfg      指向纯跟踪配置的指针
 * @return 转向角 (rad), 若找不到前视点则返回 0.
 */
double pp_compute_delta(const vehicle_state_t* state,
                        const vehicle_params_t* vparams,
                        const lane_model_t* lane,
                        const pp_config_t* cfg) {
    /* Step 1: 计算前瞻距离 */
    const double abs_v = (state->v < 0.0) ? -state->v : state->v;
    double L = cfg->lookahead_gain * abs_v;
    if (L < cfg->min_lookahead)
        L = cfg->min_lookahead;
    if (L > cfg->max_lookahead)
        L = cfg->max_lookahead;

    /* Step 2: 在车道上查找前视点 */
    double gx, gy;
    const int found = lane_find_lookahead_point(lane,
                         state->x, state->y, L, &gx, &gy);
    if (!found)
        return 0.0;

    /* Step 3: 计算朝向矢量与车辆航向的夹角 */
    const double dx = gx - state->x;
    const double dy = gy - state->y;
    const double theta_c = normalize_angle(atan2(dy, dx) - state->theta);

    /* Step 4: 纯跟踪转向公式 */
    return atan2(2.0 * L * sin(theta_c), vparams->wheelbase);
}