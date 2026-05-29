#include "simulation.h"
#include "simulation/speed_pid.h"
#include "controller/cross_error.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * 仿真主循环: 初始化 → 控制 → 积分 → 记录日志
 *
 * @param cfg 指向仿真配置的指针
 */
void simulation_run(const sim_config_t* cfg) {
    /* Step 1: 打开 CSV 输出文件 (输出到 test/ 目录) */
    FILE* fp = fopen("test/trajectory.csv", "w");
    if (fp != NULL) {
        fprintf(fp, "step,x,y,v,delta,cross_err\n");
    }

    /* Step 2: 初始化各模块 */
    vehicle_state_t   state;
    vehicle_params_t  vparams;
    lane_model_t      lane;
    pp_config_t       pp_cfg;

    vehicle_state_init(&state);
    vehicle_params_default(&vparams);
    pp_config_default(&pp_cfg);

    /* Step 3: 生成道路 */
    if (cfg->road_type == 0) {
        lane_generate_arc(&lane, ARC_RADIUS, ARC_RANGE_RAD, ARC_NUM_POINTS);
        printf("[Sim] Road type: arc (R=%.1fm, full circle)\n", ARC_RADIUS);
    } else {
        lane_generate_sine(&lane, SINE_LENGTH, SINE_AMPLITUDE,
                           SINE_FREQ, SINE_NUM_POINTS);
        printf("[Sim] Road type: sine (A=%.1fm, freq=%.2f, len=%.0fm)\n",
               SINE_AMPLITUDE, SINE_FREQ, SINE_LENGTH);
    }

    /* Step 4: 设置初始状态 */
    state.x = lane.points[0].x;
    state.y = lane.points[0].y + 0.5;
    state.theta = 0.0;
    state.v = cfg->target_speed * 0.3;
    state.delta = 0.0;
    state.delta_prev = 0.0;

    printf("[Sim] dt=%.3f s, steps=%d, target_v=%.1f m/s\n",
           cfg->dt, cfg->num_steps, cfg->target_speed);
    printf("%-6s %-12s %-12s %-8s %-8s %-8s\n",
           "Step", "x(m)", "y(m)", "v(m/s)", "delta", "cross_err");

    /* Step 5: 主控制循环 */
    for (int step = 0; step < cfg->num_steps; step++) {
        /* 5.1: 速度控制 */
        const double a = simulation_speed_pid(state.v,
                                              cfg->target_speed,
                                              vparams.max_a);
        state.v += a * cfg->dt;

        /* 5.2: 速度钳位 */
        if (state.v < 0.0)
            state.v = 0.0;
        if (state.v > vparams.max_v)
            state.v = vparams.max_v;

        /* 5.3: 纯跟踪控制 */
        state.delta = pp_compute_delta(&state, &vparams, &lane, &pp_cfg);

        /* 5.4: 转向速率限制 */
        const double max_delta_change = vparams.delta_rate * cfg->dt;
        vehicle_delta_rate_limit(&state, max_delta_change);

        /* 5.5: 计算横向误差 */
        const double cross_err = compute_cross_error(&state, &lane);

        /* 5.6: 打印和记录 (按配置间隔) */
        if (step % OUTPUT_CSV_INTERVAL == 0 || step == cfg->num_steps - 1) {
            printf("%-6d %-12.3f %-12.3f %-8.2f %-8.4f %-8.4f\n",
                   step, state.x, state.y, state.v, state.delta, cross_err);
        }

        if (fp != NULL) {
            fprintf(fp, "%d,%.6f,%.6f,%.4f,%.6f,%.6f\n",
                    step, state.x, state.y, state.v, state.delta, cross_err);
        }

        /* 5.7: 车辆模型前向积分 */
        vehicle_step(&state, &vparams, cfg->dt);
    }

        /* Step 6: 保存车道点数据 (输出到 test/ 目录) */
    FILE* lf = fopen("test/lane.csv", "w");
    if (lf != NULL) {
        fprintf(lf, "x,y\n");
        for (size_t i = 0; i < lane.count; i++) {
            fprintf(lf, "%.6f,%.6f\n", lane.points[i].x, lane.points[i].y);
        }
        fclose(lf);
    }

        /* Step 7: 清理 */
    if (fp != NULL) {
        fclose(fp);
        printf("[Sim] Data saved to test/trajectory.csv, test/lane.csv\n");
    }
    lane_model_free(&lane);
    printf("[Sim] Done. Final position: (%.3f, %.3f)\n", state.x, state.y);
}