#include "lane/model.h"
#include <math.h>

/*
 * 生成一条圆弧车道 (用于仿真)
 *   半径 radius, 圆心角 range_rad, 起点在原点朝 x 轴正向
 *
 * @param lane       指向车道模型的指针
 * @param radius     圆弧半径 (m)
 * @param range_rad  圆心角范围 (rad)
 * @param num_points 采样点数
 */
void lane_generate_arc(lane_model_t* lane,
                       double radius, double range_rad,
                       size_t num_points) {
    lane_model_init(lane);

    for (size_t i = 0; i < num_points; i++) {
        const double theta = ((double)i / (num_points - 1)) * range_rad;
        /* 圆弧: 圆心在 (0, radius), 起点在原点 */
        const double x = radius * sin(theta);
        const double y = radius * (1.0 - cos(theta));
        lane_model_push(lane, x, y);
    }
}