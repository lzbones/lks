#include "controller/cross_error.h"
#include <math.h>

/*
 * 计算车辆到车道中心线的横向误差 (Frenet 坐标系)
 *   1. 找到纵向最近的点索引 near_idx
 *   2. 用该点的切线方向作为纵向轴, 法向为横向轴
 *   3. 横向误差 = (vehicle - lane_point) · n_hat
 *
 * @param state 指向车辆状态的指针
 * @param lane  指向车道模型的指针
 * @return 横向误差 (m), 正值表示在车道左侧, 负值表示右侧
 */
double compute_cross_error(const vehicle_state_t* state,
                           const lane_model_t* lane) {
    if (lane->count < 2) {
        const double dx = state->x - lane->points[0].x;
        const double dy = state->y - lane->points[0].y;
        return sqrt(dx * dx + dy * dy);
    }

    /* Step 1: 找纵向最近点 */
    int near_idx = 0;
    double near_dist_sq = 1e18;

    for (size_t i = 0; i < lane->count; i++) {
        const double dx = lane->points[i].x - state->x;
        const double dy = lane->points[i].y - state->y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < near_dist_sq) {
            near_dist_sq = d2;
            near_idx = (int)i;
        }
    }

    /* Step 2: 计算切线和法线方向 */
    int next_idx = near_idx + 1;
    if (next_idx >= (int)lane->count) {
        next_idx = (int)(lane->count - 2);
        if (next_idx < 0) next_idx = 0;
    }

    const double tx = lane->points[next_idx].x - lane->points[near_idx].x;
    const double ty = lane->points[next_idx].y - lane->points[near_idx].y;
    const double t_len = sqrt(tx * tx + ty * ty);

    if (t_len < 1e-6) {
        return sqrt(near_dist_sq);
    }

    /* 单位切线 */
    const double utx = tx / t_len;
    const double uty = ty / t_len;

    /* 左手法向: n = (-ty, tx) */
    const double nx = -uty;
    const double ny = utx;

    /* Step 3: 投影到法向上 */
    const double dx = state->x - lane->points[near_idx].x;
    const double dy = state->y - lane->points[near_idx].y;
    return dx * nx + dy * ny;
}