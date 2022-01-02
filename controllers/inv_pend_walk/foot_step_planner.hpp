#ifndef FOOT_STEP_PLANNER_H_
#define FOOT_STEP_PLANNER_H_

#ifdef PLANNER_DEBUG
static constexpr bool planner_debug = true;
#else
static constexpr bool planner_debug = false;
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <utility>
#include <iostream>
#include <vector>

static constexpr double dt = 0.01;            // sampling period (s)
static constexpr double zh = 0.01;            // height of CoM (m)
static constexpr double g = 9.8;             // gravity (m/s^2)
static constexpr double MAX_X_STRIDE = 0.015; //(m)

struct FootPrint
{
    double time_;
    double x_;
    double y_;
    bool support_foot_is_right;
};

std::vector<FootPrint> foot_plan = {{0, 0, 0, true}, {0.8, 0.0, 0.2, false}, {1.6, 0.3, 0.0, true}, {2.4, 0.6, 0.2, false}, {3.2, 0.9, 0.0, true}, {4.0, 0.9, 0.2, false}, {100.0, 0.9, 0.2, true}};

/**
 * @brief foot step planner.左右の脚で踏み出すのを1セットで1歩とする.
 *
 * @param x_destination (m) 目標地点:X.ロボット進行方向
 * @param y_destination (m) 目標地点:Y
 * @param x_stride (m) 1歩の大きさ.片脚の歩幅=x_stride/2.
 * @param support_time (s) 歩行周期.(=支持脚が支持している時間:Tsup)
 * @param steps (none) 歩数.目標地点まで何歩で進むか.0以下に指定すると最大のストライドで進む.
 * @return std::vector<FootPrint>
 */
std::vector<FootPrint> footStepPlanner(const double &x_destination, const double &y_destination, const double &x_stride, const double &support_time = (320.0 / 1000.0), const int32_t &steps = 0)
{
    const double stride_ = [=]() -> int32_t
    {
        if (steps < 1)
        {
            return std::min(x_stride, MAX_X_STRIDE);
        }
        else
        {
            return std::min(MAX_X_STRIDE, x_destination / steps);
        }
    }();
    const int32_t steps_ = x_destination / stride_;
    double x = 0, y = 0, xi = 0, yi = 0;
    double xd = 0,yd = 0,xdi = 0,ydi = 0;
    double Tc = std::sqrt(zh/g); //時定数
    double t0 = 0;
    int_fast64_t sampling_count = 0; 
    for (double t = 0.0; t < (support_time * (steps * 2 + 1));)
    {
        for (double t_tmp = 0.0; t_tmp < support_time; t_tmp += dt, t += dt)
        {
            using namespace std;
            x = (xi - px) * cosh((t - t0)/Tc) + Tc * xdi * sinh((t - t0)/Tc) + px;
            y = (yi - py) * cosh((t - t0)/Tc) + Tc * ydi * sinh((t - t0)/Tc) + py;
            xd = (xi - px)/Tc * sinh((t - t0)/Tc)
        }
        t0 = t;
        ++sampling_count;
    }
}

#endif //  FOOT_STEP_PLANNER_H_

/* memo
 * 状態をなるべく持たせない.
 * 取り敢えず歩ける事を示したいのでなるべく簡便な実装にする。
 * 一連の重心軌道を全て生成してしまってそこからどうにかするので良い
 *
 *
 *
 *
 */