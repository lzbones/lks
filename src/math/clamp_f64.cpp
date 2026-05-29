#include "math/clamp_f64.h"

/*
 * 将 double 值限制在 [lo, hi] 区间内
 *
 * @param val 输入值
 * @param lo 下限
 * @param hi 上限
 * @return 截断后的值
 */
double clamp_f64(double val, double lo, double hi) {
    const double result = (val < lo) ? lo : ((val > hi) ? hi : val);
    return result;
}