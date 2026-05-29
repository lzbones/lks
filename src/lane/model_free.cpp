#include "lane/model.h"
#include <stdlib.h>

/*
 * 释放车道内存
 *
 * @param lane 指向车道模型的指针
 */
void lane_model_free(lane_model_t* lane) {
    free(lane->points);
    lane->points = NULL;
    lane->count = 0;
}