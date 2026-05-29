#pragma once

/*
 * 车道模型接口定义
 */

#include "lks_config.h"
#include <stddef.h>

/*
 * 道路中心线上的一个点
 */
typedef struct {
    double x;
    double y;
} lane_point_t;

/*
 * 车道模型: 由一组离散的中心线点描述
 */
typedef struct {
    lane_point_t* points;   /* 动态数组 (malloc) */
    size_t        count;    /* 点数 */
} lane_model_t;

/* ---- 函数声明 ---- */

/*
 * 初始化空车道
 */
void lane_model_init(lane_model_t* lane);

/*
 * 释放车道内存
 */
void lane_model_free(lane_model_t* lane);

/*
 * 添加一个点到车道末尾
 */
void lane_model_push(lane_model_t* lane, double x, double y);

/*
 * 生成一条正弦弯曲的车道 (用于仿真)
 *   y = amplitude * sin(freq * x),  x in [0, length]
 */
void lane_generate_sine(lane_model_t* lane,
                        double length, double amplitude,
                        double freq, size_t num_points);

/*
 * 生成一条圆弧车道 (用于仿真)
 *   半径 radius, 圆心角 range_rad, 起点在原点朝 x 轴正向
 */
void lane_generate_arc(lane_model_t* lane,
                       double radius, double range_rad,
                       size_t num_points);

/*
 * 在车道上查找距离车辆前方 closest longitudinal distance
 * 的前视点 (lookahead point), 返回该点的索引.
 * 若找不到则返回 -1.
 */
int lane_find_lookahead_index(const lane_model_t* lane,
                              double vehicle_x, double vehicle_y,
                              double lookahead_dist);

/*
 * 在车道上查找前视点的实际坐标 (带线性插值).
 * 返回值: 通过指针返回 (gx, gy), 成功返回 1, 失败返回 0.
 */
int lane_find_lookahead_point(const lane_model_t* lane,
                              double vehicle_x, double vehicle_y,
                              double lookahead_dist,
                              double* gx, double* gy);