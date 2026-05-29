#include "lane/model.h"
#include <math.h>

/*
 * 查找前视点索引:
 *   1. 先找到距离车辆最近的车道点索引 near_idx
 *   2. 从 near_idx 开始沿车道向前累加弧长, 直到累计距离 >= lookahead_dist
 *
 * @param lane           指向车道模型的指针
 * @param vehicle_x      车辆横坐标
 * @param vehicle_y      车辆纵坐标
 * @param lookahead_dist 前瞻距离 (m)
 * @return 找到的前视点索引 (>= near_idx), 失败返回 -1.
 */
int lane_find_lookahead_index(const lane_model_t* lane,
                              double vehicle_x, double vehicle_y,
                              double lookahead_dist) {
    if (lane->count == 0)
        return -1;

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

    /* Step 2: 从最近点向前累加弧长 */
    double accumulated = sqrt(near_dist_sq);

    for (size_t i = (size_t)(near_idx + 1); i < lane->count; i++) {
        const double seg_dx = lane->points[i].x - lane->points[i - 1].x;
        const double seg_dy = lane->points[i].y - lane->points[i - 1].y;
        const double seg_len = sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

        accumulated += seg_len;

        if (accumulated >= lookahead_dist)
            return (int)i;
    }

    /* 走到最后一个点仍不够, 返回最后一个点的索引 */
    return (int)(lane->count - 1);
}