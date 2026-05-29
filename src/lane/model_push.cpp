#include "lane/model.h"
#include <stdlib.h>

/*
 * 添加一个点到车道末尾 (自动扩容)
 *
 * @param lane 指向车道模型的指针
 * @param x    点的横坐标
 * @param y    点的纵坐标
 */
void lane_model_push(lane_model_t* lane, double x, double y) {
    const size_t init_cap = LANE_INIT_CAP;

    /* Step 1: 需要分配新内存? */
    if (lane->points == NULL) {
        lane_point_t* new_points = (lane_point_t*)malloc(
            init_cap * sizeof(lane_point_t));
        if (new_points == NULL) return;
        lane->points = new_points;
    }

    /* Step 2: 已满, 需要扩容? */
    else if (lane->count >= init_cap) {
        size_t current_cap = init_cap;
        while (current_cap <= lane->count)
            current_cap *= 2;

        lane_point_t* new_points = (lane_point_t*)realloc(
            lane->points, current_cap * sizeof(lane_point_t));
        if (new_points == NULL) return;
        lane->points = new_points;
    }

    /* Step 3: 添加点 */
    lane->points[lane->count].x = x;
    lane->points[lane->count].y = y;
    lane->count++;
}