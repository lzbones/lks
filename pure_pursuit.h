#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "vehicle_model.h"
#include "lane_model.h"
#include "config.h"

/*
 * 纯跟踪控制器参数
 */
typedef struct {
    double lookahead_gain;   /* 前瞻距离 = gain * speed (s) */
    double min_lookahead;    /* 最小前瞻距离 (m) */
    double max_lookahead;    /* 最大前瞻距离 (m) */
    double k_cross;          /* 横向误差增益 (P项) */
} PurePursuitConfig;

/*
 * 使用默认参数初始化控制器配置
 */
void pure_pursuit_config_default(PurePursuitConfig* cfg);

/*
 * 纯跟踪控制: 计算转向角 delta.
 *
 * 算法:
 *   1. 根据当前速度计算前瞻距离 L = max(min_lookahead, gain * v)
 *   2. 在车道中心线上找到距车辆约 L 米的前视点 (gx, gy)
 *   3. delta = 2 * L_c * sin(theta_c) / wheelbase
 *      其中 theta_c 为朝向矢量与车辆航向的夹角
 *
 * 输出 delta 单位: rad
 * 若找不到前视点则返回 0.
 */
double pure_pursuit_compute_delta(
    const VehicleState* state,
    const VehicleParams* vparams,
    const LaneModel* lane,
    const PurePursuitConfig* cfg);

#endif /* PURE_PURSUIT_H */

