#pragma once

/*
 * 仿真引擎接口定义
 */

#include "vehicle/state.h"
#include "lane/model.h"
#include "controller/pure_pursuit.h"

/*
 * 仿真配置
 */
typedef struct {
    double dt;             /* 控制周期 (s) */
    int    num_steps;      /* 总步数 */
    double target_speed;   /* 期望巡航速度 (m/s) */
    int    road_type;      /* 0=圆弧, 1=正弦弯 */
} sim_config_t;

/*
 * 仿真主循环: 初始化 → 控制 → 积分 → 记录日志
 */
void simulation_run(const sim_config_t* cfg);