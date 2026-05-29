#include "simulation.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * 简单的速度跟踪: P 控制器调节加速度
 */
static double compute_acceleration(double current_v,
                                   double target_v,
                                   double max_a)
{
    double a = (target_v - current_v) * SPEED_P_GAIN;
    if (a > max_a)   a = max_a;
    if (a < -max_a)  a = -max_a;
    return a;
}

/*
 * 计算车辆到车道中心线的横向误差 (Frenet 坐标系)
 *   1. 找到纵向最近的点索引 near_idx
 *   2. 用该点的切线方向作为纵向轴, 法向为横向轴
 *   3. 横向误差 = (vehicle - lane_point) · n_hat
 */
static double compute_cross_error(const VehicleState* state,
                                  const LaneModel* lane)
{
    if (lane->count < 2)
        return sqrt(1e18);

    /* Step 1: 找纵向最近点 */
    int near_idx = 0;
    double near_dist_sq = 1e18;

    for (size_t i = 0; i < lane->count; i++) {
        double dx = lane->points[i].x - state->x;
        double dy = lane->points[i].y - state->y;
        double d2 = dx * dx + dy * dy;
        if (d2 < near_dist_sq) {
            near_dist_sq = d2;
            near_idx = (int)i;
        }
    }

    /* Step 2: 计算切线和法线方向 */
    /* 边界检查: near_idx + 1 不能超过车道点数 */
    int next_idx = near_idx + 1;
    if (next_idx >= (int)lane->count) {
        /* 最近点就是最后一个点, 用最后一段的切线方向 */
        next_idx = (int)(lane->count - 2);
        if (next_idx < 0) next_idx = 0;
    }

    double tx = lane->points[next_idx].x - lane->points[near_idx].x;
    double ty = lane->points[next_idx].y - lane->points[near_idx].y;
    double t_len = sqrt(tx * tx + ty * ty);

    if (t_len < 1e-6)
        return sqrt(near_dist_sq);

    /* 单位切线 */
    tx /= t_len;
    ty /= t_len;

    /* 左手法向: n = (-ty, tx) */
    double nx = -ty;
    double ny = tx;

    /* Step 3: 投影到法向上 */
    double dx = state->x - lane->points[near_idx].x;
    double dy = state->y - lane->points[near_idx].y;
    return dx * nx + dy * ny;
}

void simulation_run(const SimConfig* cfg) {
    /* 打开 CSV 输出文件 */
    FILE *fp = fopen("trajectory.csv", "w");
    if (fp) {
        fprintf(fp, "step,x,y,v,delta,cross_err\n");
    }

    /* ========== 初始化 ========== */
    VehicleState   state;
    VehicleParams  vparams;
    LaneModel      lane;
    PurePursuitConfig pp_cfg;

    vehicle_state_init(&state);
    vehicle_params_default(&vparams);
    pure_pursuit_config_default(&pp_cfg);

    /* 生成道路 */
    if (cfg->road_type == 0) {
        /* 圆弧道路: 半径 80m, 弯角 2pi (一整圈) */
        lane_generate_arc(&lane, CFG_RADIUS, CFG_RANGE_RAD, CFG_NUM_POINTS);
        printf("[Sim] Road type: arc (R=%.1fm, full circle)\n",
               CFG_RADIUS);
    } else {
        /* 正弦弯道: 振幅 5m, 波长约 63m, 总长 2000m */
        lane_generate_sine(&lane, CFG_SINE_LENGTH, CFG_SINE_AMPLITUDE,
                           CFG_SINE_FREQ, CFG_SINE_NUM_POINTS);
        printf("[Sim] Road type: sine (A=%.1fm, freq=%.2f, len=%.0fm)\n",
               CFG_SINE_AMPLITUDE, CFG_SINE_FREQ, CFG_SINE_LENGTH);
    }

    /* 车辆初始位置放在车道起点附近 */
    state.x = lane.points[0].x;
    state.y = lane.points[0].y + 0.5;  /* 给一点初始横向偏移 */
    state.theta = 0.0;
    state.v = cfg->target_speed * 0.3;
    state.delta = 0.0;
    state.delta_prev = 0.0;

    printf("[Sim] dt=%.3f s, steps=%d, target_v=%.1f m/s\n",
           cfg->dt, cfg->num_steps, cfg->target_speed);
    printf("%-6s %-12s %-12s %-8s %-8s %-8s\n",
           "Step", "x(m)", "y(m)", "v(m/s)", "delta", "cross_err");

    /* ========== 主控制循环 ========== */
    for (int step = 0; step < cfg->num_steps; step++) {
        /* 1. 速度控制: 计算加速度并更新速度 */
        double a = compute_acceleration(state.v,
                                        cfg->target_speed,
                                        vparams.max_a);
        state.v += a * cfg->dt;

        /* 速度钳位: 不允许负速度和超过最大速度 */
        if (state.v < 0.0) state.v = 0.0;
        if (state.v > vparams.max_v) state.v = vparams.max_v;

        /* 2. 纯跟踪: 计算转向角 */
        state.delta = pure_pursuit_compute_delta(
            &state, &vparams, &lane, &pp_cfg);

        /* 3. 转向速率限制: |delta - delta_prev| <= delta_rate * dt */
        double max_delta_change = vparams.delta_rate * cfg->dt;
        vehicle_limit_delta_rate(&state, max_delta_change);

        /* 4. 记录当前横向误差 (按配置间隔打印) */
        double cross_err = compute_cross_error(&state, &lane);

        if (step % OUTPUT_CSV_INTERVAL == 0 || step == cfg->num_steps - 1) {
            printf("%-6d %-12.3f %-12.3f %-8.2f %-8.4f %-8.4f\n",
                   step, state.x, state.y,
                   state.v, state.delta, cross_err);
        }

        /* 写 CSV */
        if (fp) {
            fprintf(fp, "%d,%.6f,%.6f,%.4f,%.6f,%.6f\n",
                    step, state.x, state.y,
                    state.v, state.delta, cross_err);
        }

        /* 5. 车辆模型前向积分 */
        vehicle_step(&state, &vparams, cfg->dt);
    }

    /* 保存车道点数据 */
    FILE *lf = fopen("lane.csv", "w");
    if (lf) {
        fprintf(lf, "x,y\n");
        for (size_t i = 0; i < lane.count; i++) {
            fprintf(lf, "%.6f,%.6f\n", lane.points[i].x,
                    lane.points[i].y);
        }
        fclose(lf);
    }

    /* ========== 清理 ========== */
    if (fp) {
        fclose(fp);
        printf("[Sim] Data saved to trajectory.csv, lane.csv\n");
    }
    lane_model_free(&lane);
    printf("[Sim] Done. Final position: (%.3f, %.3f)\n",
           state.x, state.y);
}