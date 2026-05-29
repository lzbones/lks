#include "lane/model.h"

/*
 * 初始化空车道
 *
 * @param lane 指向车道模型的指针
 */
void lane_model_init(lane_model_t* lane) {
    lane->points = NULL;
    lane->count = 0;
}