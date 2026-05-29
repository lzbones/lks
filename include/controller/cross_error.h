#pragma once

/*
 * 横向误差计算接口
 */

#include "vehicle/state.h"
#include "lane/model.h"

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
                           const lane_model_t* lane);