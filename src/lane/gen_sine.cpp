#include "lane/model.h"
#include <math.h>

/*
 * 生成一条正弦弯曲的车道 (用于仿真)
 *   y = amplitude * sin(freq * x),  x in [0, length]
 *
 * @param lane       指向车道模型的指针
 * @param length     总长度 (m)
 * @param amplitude  振幅 (m)
 * @param freq       频率 (rad/m)
 * @param num_points 采样点数
 */
void lane_generate_sine(lane_model_t* lane,
                        double length, double amplitude,
                        double freq, size_t num_points) {
    lane_model_init(lane);

    for (size_t i = 0; i < num_points; i++) {
        const double x = ((double)i / (num_points - 1)) * length;
        const double y = amplitude * sin(freq * x);
        lane_model_push(lane, x, y);
    }
}