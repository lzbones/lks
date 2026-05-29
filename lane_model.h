#ifndef LANE_MODEL_H
#define LANE_MODEL_H

#include <stddef.h>

/*
 * 道路中心线上的一个点
 */
typedef struct {
    double x;
    double y;
} LanePoint;

/*
 * 车道模型: 由一组离散的中心线点描述
 */
typedef struct {
    LanePoint* points;   // 动态数组 (malloc)
    size_t     count;    // 点数
} LaneModel;

/*
 * 初始化空车道
 */
void lane_model_init(LaneModel* lane);

/*
 * 释放车道内存
 */
void lane_model_free(LaneModel* lane);

/*
 * 添加一个点到车道末尾
 */
void lane_model_push(LaneModel* lane, double x, double y);

/*
 * 生成一条正弦弯曲的车道 (用于仿真)
 *   y = amplitude * sin(freq * x),  x in [0, length]
 */
void lane_generate_sine(LaneModel* lane,
                        double length, double amplitude,
                        double freq, size_t num_points);

/*
 * 生成一条圆弧车道 (用于仿真)
 *   半径 radius, 圆心角 range_rad, 起点在原点朝 x 轴正向
 */
void lane_generate_arc(LaneModel* lane,
                       double radius, double range_rad,
                       size_t num_points);

/*
 * 在车道上查找距离车辆前方 closest longitudinal distance
 * 的前视点 (lookahead point), 返回该点的索引.
 * 若找不到则返回 -1.
 */
int lane_find_lookahead(const LaneModel* lane,
                        double vehicle_x, double vehicle_y,
                        double lookahead_dist);

/*
 * 在车道上查找前视点的实际坐标 (带线性插值).
 * 返回值: 通过 struct 返回 (gx, gy), 成功返回 1, 失败返回 0.
 */
int lane_find_lookahead_point(const LaneModel* lane,
                              double vehicle_x, double vehicle_y,
                              double lookahead_dist,
                              double* gx, double* gy);

#endif /* LANE_MODEL_H */

