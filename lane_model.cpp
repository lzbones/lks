#include "lane_model.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LANE_INIT_CAP 64

void lane_model_init(LaneModel* lane) {
    lane->points = NULL;
    lane->count = 0;
}

void lane_model_free(LaneModel* lane) {
    free(lane->points);
    lane->points = NULL;
    lane->count = 0;
}

void lane_model_push(LaneModel* lane, double x, double y) {
    /* 需要分配新内存? */
    if (lane->points == NULL) {
        lane->points = (LanePoint*)malloc(LANE_INIT_CAP * sizeof(LanePoint));
        if (lane->points == NULL) return;  /* malloc 失败保护 */
    }
    /* 已满, 需要扩容? */
    else if (lane->count >= LANE_INIT_CAP) {
        size_t current_cap = LANE_INIT_CAP;
        while (current_cap <= (size_t)lane->count)
            current_cap *= 2;
        LanePoint* new_points = (LanePoint*)realloc(
                lane->points, current_cap * sizeof(LanePoint));
        if (new_points == NULL) return;  /* realloc 失败保护 */
        lane->points = new_points;
    }

    lane->points[lane->count].x = x;
    lane->points[lane->count].y = y;
    lane->count++;
}

void lane_generate_sine(LaneModel* lane,
                        double length, double amplitude,
                        double freq, size_t num_points) {
    lane_model_init(lane);
    for (size_t i = 0; i < num_points; i++) {
        double x = (double)i / (num_points - 1) * length;
        double y = amplitude * sin(freq * x);
        lane_model_push(lane, x, y);
    }
}

void lane_generate_arc(LaneModel* lane,
                       double radius, double range_rad,
                       size_t num_points) {
    lane_model_init(lane);
    for (size_t i = 0; i < num_points; i++) {
        double theta = (double)i / (num_points - 1) * range_rad;
        /* 圆弧: 圆心在 (0, radius), 起点在原点 */
        double x = radius * sin(theta);
        double y = radius * (1.0 - cos(theta));
        lane_model_push(lane, x, y);
    }
}

/*
 * 查找前视点:
 *   1. 先找到距离车辆最近的车道点索引 near_idx
 *   2. 从 near_idx 开始沿车道向前累加弧长, 直到累计距离 >= lookahead_dist
 *   3. 在跨越阈值的前后两点之间做线性插值, 返回前视点的坐标
 *
 * 返回值: 找到的前视点索引 (>= near_idx), 失败返回 -1.
 */
int lane_find_lookahead(const LaneModel* lane,
                        double vehicle_x, double vehicle_y,
                        double lookahead_dist) {
    if (lane->count == 0)
        return -1;

    /* Step 1: 找最近点 */
    int near_idx = 0;
    double near_dist_sq = 1e18;

    for (size_t i = 0; i < lane->count; i++) {
        double dx = lane->points[i].x - vehicle_x;
        double dy = lane->points[i].y - vehicle_y;
        double d2 = dx * dx + dy * dy;
        if (d2 < near_dist_sq) {
            near_dist_sq = d2;
            near_idx = (int)i;
        }
    }

    /* Step 2: 从最近点向前累加弧长 */
    double accumulated = sqrt(near_dist_sq);  // 起始距离

    for (size_t i = (size_t)(near_idx + 1);
         i < lane->count;
         i++) {
        /* 计算相邻车道点之间的段长 */
        double seg_dx = lane->points[i].x - lane->points[i - 1].x;
        double seg_dy = lane->points[i].y - lane->points[i - 1].y;
        double seg_len = sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

        accumulated += seg_len;

        if (accumulated >= lookahead_dist) {
            return (int)i;  // 前视点落在这一段, 取终点即可
        }
    }

    /* 走到最后一个点仍不够, 返回最后一个点的索引 */
    return (int)(lane->count - 1);
}

/*
 * 查找前视点坐标 (带线性插值):
 *   1. 调用 lane_find_lookahead 找到跨越前瞻距离的段索引 idx
 *   2. 在 points[idx-1] 和 points[idx] 之间按剩余距离比例插值
 */
int lane_find_lookahead_point(const LaneModel* lane,
                              double vehicle_x, double vehicle_y,
                              double lookahead_dist,
                              double* gx, double* gy)
{
    if (lane->count == 0 || lane->count < 2)
        return 0;

    /* Step 1: 找最近点 */
    int near_idx = 0;
    double near_dist_sq = 1e18;

    for (size_t i = 0; i < lane->count; i++) {
        double dx = lane->points[i].x - vehicle_x;
        double dy = lane->points[i].y - vehicle_y;
        double d2 = dx * dx + dy * dy;
        if (d2 < near_dist_sq) {
            near_dist_sq = d2;
            near_idx = (int)i;
        }
    }

    /* Step 2: 从最近点向前累加弧长, 找到目标段 */
    double accumulated = sqrt(near_dist_sq);

    for (size_t i = (size_t)(near_idx + 1); i < lane->count; i++) {
        double seg_dx = lane->points[i].x - lane->points[i - 1].x;
        double seg_dy = lane->points[i].y - lane->points[i - 1].y;
        double seg_len = sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

        if (accumulated + seg_len >= lookahead_dist) {
            /* 目标点落在 [i-1, i] 这一段, 线性插值 */
            double remaining = lookahead_dist - accumulated;
            double ratio = (seg_len > 1e-10) ? (remaining / seg_len) : 0.0;
            if (ratio < 0.0) ratio = 0.0;
            if (ratio > 1.0) ratio = 1.0;

            *gx = lane->points[i - 1].x + ratio * seg_dx;
            *gy = lane->points[i - 1].y + ratio * seg_dy;
            return 1;
        }

        accumulated += seg_len;
    }

    /* 走到最后一个点仍不够, 返回终点 */
    *gx = lane->points[lane->count - 1].x;
    *gy = lane->points[lane->count - 1].y;
    return 1;
}

