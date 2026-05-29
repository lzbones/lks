#pragma once

/*
 * LKS (Lane Keeping System) 全局配置参数
 * 集中管理所有魔法数字，便于调参和维护
 */

/* ---- 仿真配置 ---- */
#define LKS_DT                0.05    /* 控制周期 (s), 20 Hz */
#define LKS_NUM_STEPS         4000    /* 总步数 = 200 秒仿真 */
#define LKS_TARGET_SPEED      15.0    /* 目标速度 (m/s) = 54 km/h */
#define LKS_ROAD_TYPE_ARC     0       /* 圆弧道路 */
#define LKS_ROAD_TYPE_SINE    1       /* 正弦弯道 */

/* ---- 车辆参数 ---- */
#define LKS_WHEELBASE         2.7     /* 轴距 (m) */
#define LKS_MAX_DELTA         (M_PI / 4)   /* 最大转向角 = 45° */
#define LKS_DELTA_RATE_LIMIT  2.0     /* 转向速率上限 (rad/s) */
#define LKS_MAX_V             30.0    /* 最大速度 (m/s) = 108 km/h */
#define LKS_MIN_V             0.0     /* 最小速度 */
#define LKS_MAX_A             3.0     /* 最大加速度 (m/s²) */

/* ---- Pure Pursuit 控制器参数 ---- */
#define PP_LOOKAHEAD_GAIN     0.4     /* 前瞻距离增益 = gain × speed (s) */
#define PP_MIN_LOOKAHEAD      2.0     /* 最小前瞻距离 (m) */
#define PP_MAX_LOOKAHEAD      8.0     /* 最大前瞻距离 (m) */
#define PP_K_CROSS            0.5     /* 横向误差补偿增益 (P项) */

/* ---- 速度控制器参数 ---- */
#define SPEED_P_GAIN          2.0     /* 速度 P 控制器比例增益 */

/* ---- 车道模型参数 ---- */
#define LANE_INIT_CAP         64      /* 初始容量 */

/* ---- 道路生成参数（圆弧模式）---- */
#define ARC_RADIUS            80.0    /* 圆弧半径 (m) */
#define ARC_RANGE_RAD         (2.0 * M_PI)  /* 圆心角 = 整圆 */
#define ARC_NUM_POINTS        10000   /* 采样点数 */

/* ---- 道路生成参数（正弦模式）---- */
#define SINE_LENGTH           2000.0  /* 总长度 (m) */
#define SINE_AMPLITUDE        5.0     /* 振幅 (m) */
#define SINE_FREQ             0.1     /* 频率 (rad/m), 波长 = 2π/0.1 ≈ 63m */
#define SINE_NUM_POINTS       10000   /* 采样点数 */

/* ---- 输出配置 ---- */
#define OUTPUT_CSV_INTERVAL   50      /* CSV 打印间隔步数 */