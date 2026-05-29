#include "lane/model.h"
#include <math.h>

/*
 * 查找前视点坐标 (带线性插值):
 *   1. 先找到距离车辆最近的车道点索引 near_idx
 *   2. 从 near_idx 开始沿车道向前累加弧长, 找到跨越前瞻距离的目标段
 *   3. 在目标段内按剩余距离比例做线性插值
 *
 * @param lane           指向车道模型的指针
 * @param vehicle_x      车辆横坐标
 * @param vehicle_y      车辆纵坐标
 * @param lookahead_dist 前瞻距离 (m)
 * @param gx             输出: 前视点横坐标
 * @param gy             输出: 前视点纵坐标
 * @return 成功返回 1, 失败返回 0.
 */
int lane_find_lookahead_point(const lane_model_t* lane,
                              double vehicle_x, double vehicle_y,
                              double lookahead_dist,
                              double* gx, double* gy) {
    if (lane->count == 0 || lane->count < 2)
        return 0;

    /* Step 1: 找最近点 */
    int near_idx = 0;
    double near_dist_sq = 1e18;

    for (size_t i = 0; i < lane->count; i++) {
        const double dx = lane->points[i].x - vehicle_x;
        const double dy = lane->points[i].y - vehicle_y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < near_dist_sq) {
            near_dist_sq = d2;
            near_idx = (int)i;
        }
    }

    /* Step 2: 从最近点向前累加弧长, 找到目标段 */
    double accumulated = sqrt(near_dist_sq);

    for (size_t i = (size_t)(near_idx + 1); i < lane->count; i++) {
        const double seg_dx = lane->points[i].x - lane->points[i - 1].x;
        const double seg_dy = lane->points[i].y - lane->points[i - 1].y;
        const double seg_len = sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

        if (accumulated + seg_len >= lookahead_dist) {
            /* 目标点落在 [i-1, i] 这一段, 线性插值 */
            const double remaining = lookahead_dist - accumulated;
            const double ratio = (seg_len > 1e-10) ? (remaining / seg_len) : 0.0;
            const double clamped_ratio = (ratio < 0.0) ? 0.0 : ((ratio > 1.0) ? 1.0 : ratio);

            *gx = lane->points[i - 1].x + clamped_ratio * seg_dx;
            *gy = lane->points[i - 1].y + clamped_ratio * seg_dy;
            return 1;
        }

        accumulated += seg_len;
    }

    /* 走到最后一个点仍不够, 返回终点 */
    *gx = lane->points[lane->count - 1].x;
    *gy = lane->points[lane->count - 1].y;
    return 1;
}