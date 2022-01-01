#ifndef FOOT_STEP_PLANNER_H_
#define FOOT_STEP_PLANNER_H_

#ifdef PLANNER_DEBUG
static constexpr bool planner_debug = true;
#else
static constexpr bool planner_debug = false;
#endif

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>




#endif //  FOOT_STEP_PLANNER_H_

/**memo
 * 状態をなるべく持たせない.
 * 取り敢えず歩ける事を示したいのでなるべく簡便な実装にする。
 * 一連の重心軌道を全て生成してしまってそこからどうにかするので良い
 * 
 * 
 * 
 * 
 * /