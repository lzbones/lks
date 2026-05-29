#include "simulation.h"
#include <stdio.h>

int main(void) {
    sim_config_t cfg;

    cfg.dt = LKS_DT;              /* 20 Hz 控制频率 */
    cfg.num_steps = LKS_NUM_STEPS;/* 仿真 200 秒 */
    cfg.target_speed = LKS_TARGET_SPEED; /* 54 km/h */
    cfg.road_type = 0;            /* 圆弧道路 */

    printf("=== Lane Keeping System (Pure Pursuit) ===\n");
    simulation_run(&cfg);

    return 0;
}