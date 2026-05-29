#include "pure_pursuit.h"
#include <math.h>

void pure_pursuit_config_default(PurePursuitConfig* cfg) {
    cfg->lookahead_gain = PP_LOOKAHEAD_GAIN;
    cfg->min_lookahead = PP_MIN_LOOKAHEAD;
    cfg->max_lookahead = PP_MAX_LOOKAHEAD;
    cfg->k_cross     = PP_K_CROSS;
}

double pure_pursuit_compute_delta(
    const VehicleState* state,
    const VehicleParams* vparams,
    const LaneModel* lane,
    const PurePursuitConfig* cfg)
{
    /* ---- Step 1: 计算前瞻距离 ---- */
    double L = cfg->lookahead_gain * fabs(state->v);
    if (L < cfg->min_lookahead)
        L = cfg->min_lookahead;
    if (L > cfg->max_lookahead)
        L = cfg->max_lookahead;

    /* ---- Step 2: 在车道上查找前视点 ---- */
    int idx = lane_find_lookahead(lane, state->x, state->y, L);
    if (idx < 0)
        return 0.0;  // 找不到目标点, 保持直行

    double gx = lane->points[idx].x;
    double gy = lane->points[idx].y;

    /* ---- Step 3: 计算朝向矢量与车辆航向的夹角 ---- */
    double dx = gx - state->x;
    double dy = gy - state->y;
    double theta_c = atan2(dy, dx) - state->theta;
    theta_c = normalize_angle(theta_c);

    /* ---- Step 4: 纯跟踪转向公式 ---- */
    double delta = atan2(2.0 * L * sin(theta_c),
                         vparams->wheelbase);

    return delta;
}

