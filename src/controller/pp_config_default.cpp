#include "controller/pure_pursuit.h"

/*
 * 使用默认参数初始化纯跟踪控制器配置
 *
 * @param cfg 指向纯跟踪配置的指针
 */
void pp_config_default(pp_config_t* cfg) {
    cfg->lookahead_gain = PP_LOOKAHEAD_GAIN;
    cfg->min_lookahead = PP_MIN_LOOKAHEAD;
    cfg->max_lookahead = PP_MAX_LOOKAHEAD;
    cfg->k_cross = PP_K_CROSS;
}