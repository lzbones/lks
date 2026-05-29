#include "simulation.h"
#include <stdio.h>

int main(void) {
    SimConfig cfg;

    cfg.dt           = CFG_DT;          /* 20 Hz 控制频率 */
    cfg.num_steps    = CFG_NUM_STEPS;   /* 仿真 200 秒 */
    cfg.target_speed = CFG_TARGET_SPEED;/* 54 km/h */
    cfg.road_type    = CFG_ROAD_TYPE_ARC;

    printf("=== Lane Keeping System (Pure Pursuit) ===\n");
    simulation_run(&cfg);

    return 0;
}