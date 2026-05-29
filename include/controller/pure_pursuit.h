#pragma once

/*
 * 纯跟踪控制器接口定义
 */

#include "vehicle/state.h"
#include "lane/model.h"
#include "lks_config.h"

/*
 * 纯跟踪控制器参数
 */
typedef struct {
    double lookahead_gain;   /* 前瞻距离 = gain × speed (s) */
    double min_lookahead;    /* 最小前瞻距离 (m) */
    double max_lookahead;    /* 最大前瞻距离 (m) */
    double k_cross;          /* 横向误差增益 (P项) */
} pp_config_t;

/* ---- 函数声明 ---- */

/*
 * 使用默认参数初始化纯跟踪控制器配置
 */
void pp_config_default(pp_config_t* cfg);

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
                        const pp_config_t* cfg);