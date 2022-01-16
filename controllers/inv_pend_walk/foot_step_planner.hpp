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
#include <fstream>
#include <utility>
#include <iostream>
#include <deque>
#include <vector>

template <class T>
static void debugPrint(T &&s,const std::string& label)
{
    if (planner_debug)
        std::cout << "Debug planner:" << label << "::"<< std::forward<T>(s) << std::endl;
}

static constexpr double dt = 4.0 / 1000.0;   // sampling period (s)
static constexpr double zh = 0.2;            // height of CoM (m)
static constexpr double g = 9.8;             // gravity (m/s^2)
static constexpr double MAX_X_STRIDE = 0.30; //(m)
static constexpr double MAX_Y_STRIDE = 0.08; //(m)
static constexpr double MIN_Y_STRIDE = 0.03; //(m)

struct FootPrint
{
    double time_;
    double x_;                  //(m)ロボットのローカル座標での足配置位置
    double y_;                  //(m)ロボットのローカル座標での足配置位置
    bool support_foot_is_right; //支持脚がどちらか
};

std::vector<FootPrint> foot_plan = {{0, 0, 0, true}, {0.8, 0.0, 0.2, false}, {1.6, 0.3, 0.0, true}, {2.4, 0.6, 0.2, false}, {3.2, 0.9, 0.0, true}, {4.0, 0.9, 0.2, false}, {100.0, 0.9, 0.2, true}};

/**
 * @brief foot step planner.左右の脚で踏み出すのを1セットで1歩とする.
 *
 * @param x_destination (m) 目標地点:X.ロボット進行方向
 * @param y_destination (m) 目標地点:Y
 * @param x_stride (m) 1歩の大きさ.遊脚が身体の前に出る長さ=x_stride/2.
 * @param support_time (s) 歩行周期.(=支持脚が支持している時間:Tsup)
 * @param steps (none) 歩数.目標地点まで何歩で進むか.0以下に指定すると最大のストライドで進む.
 * @return std::vector<FootPrint> ロボットのローカル座標で表した着地位置を返す.
 * @details 今の所x方向への直線移動しか対応していない.
 * @todo y方向の移動の実装
 */
std::vector<FootPrint> footStepPlanner(const double &x_destination, const double &y_destination, const double &x_stride, const double &support_time = (320.0 / 1000.0), const int32_t &steps = 0) noexcept
{
    const double stride_x = [=]() -> double
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
    const double stride_y = std::clamp((y_destination / steps), MIN_Y_STRIDE, MAX_Y_STRIDE);
    const int32_t steps_ = x_destination / stride_x; //(無) 歩数
    std::cout << "x_destination " << x_destination << " stride_x " << stride_x << "steps " << steps << std::endl;
    double x = 0, y = 0, xi = 0, yi = 0;     //(m) CoMのワールド座標 {CoM = Center of Mass}
    double xd = 0, yd = 0, xdi = 0, ydi = 0; // CoMの速度 v(m/s) xdot(t)
    double px = 0.0, py = 0.0;               //(m)　着地位置のワールド座標 これは実用的にはローカルの方が良いのでは？？
    constexpr double Tc = std::sqrt(zh / g); //微分方程式の時定数
    double C = 0.0;                          // cosh((t - t0) / Tc)
    double S = 0.0;                          // sinh((t - t0) / Tc)
    // const double Tsup = support_time - std::fmod(support_time, dt); //誤差を無くすため
    const double Tsup = support_time;
    int_fast64_t step_n = 0;
    debugPrint(Tsup,"Tsup");
    std::deque<std::deque<double>> result;
    std::ofstream velofs("velo.dat");
    std::vector<FootPrint> footprint_list(100);
    for (double t = 0.0; t < (Tsup * static_cast<double>(steps_ * 2 + 1)); ++step_n)
    {
        //決められた次の一歩を着く地点までの遊脚の移動を行っている時のシミュレーション---------------
        for (double t_tmp = 0.0; t_tmp < Tsup; t_tmp += dt, t += dt)
        {
            C = std::cosh(t_tmp / Tc);
            S = std::sinh(t_tmp / Tc);
            x = (xi - px) * C + Tc * xdi * S + px; // x,xdともにn歩目開始時の状態
            y = (yi - py) * C + Tc * ydi * S + py;
            xd = (xi - px) / Tc * S + xdi * C;
            yd = (yi - py) / Tc * S + ydi * C;
            result.push_back({x, y});
            velofs << t << " " << xd << " " << yd << std::endl;
        }
        xi = x;
        yi = y;
        xdi = xd;
        ydi = yd;
        //次の一歩の目標位置を計算------------------
        static constexpr double a = 10;
        static constexpr double b = 1;
        C = std::cosh(Tsup / Tc); //意味は全く無いが分かりやすさのため
        S = std::sinh(Tsup / Tc);
        const double D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);
        double x_target = 0.0, y_target = 0.0, xd_target = 0.0, yd_target = 0.0;
        double sx = 0.0, sy = 0.0; //歩行素片による位置
        if (steps_ < step_n)
        {
            sx = 0.0;
            sy = 0.0;
        }
        else
        {
            sx = stride_x / 2;
            sy = ((step_n % 2) ? 1 : -1) * stride_y / 2;
        }
        x_target = px + sx;
        y_target = py + sy;
        std::cout << "x target " << x_target << " y target " << y_target;
        xd_target = (C + 1) / (Tc * S) * sx;
        yd_target = (C - 1) / (Tc * S) * sy;
        px = -a * (C - 1) / D * (x_target - C * xi - Tc * S * xdi) - b * S / (Tc * D) * (xd_target - S * xi / Tc - C * xdi);
        py = -a * (C - 1) / D * (y_target - C * yi - Tc * S * ydi) - b * S / (Tc * D) * (yd_target - S * yi / Tc - C * ydi);
        std::cout << " px::" << px << " py::" << py << std::endl;
        static bool is_right = true;
        footprint_list.push_back({Tsup,px - xi,py - yi,is_right});
        is_right = !is_right;
        result.back().push_back(px);
        result.back().push_back(py);
    }
    std::ofstream ofs("position.dat");
    for (auto &itr : result)
    {
        for (auto &item : itr)
        {
            ofs << item << " ";
        }
        ofs << std::endl;
    }
    return footprint_list;
    // ofs.close();
}

#endif //  FOOT_STEP_PLANNER_H_

/* memo
 * 状態をなるべく持たせない.
 * 取り敢えず歩ける事を示したいのでなるべく簡便な実装にする。
 * 一連の重心軌道を全て生成してしまってそこからどうにかするので良い
 * todo 速度が極大な所のtを表示させたい
 * 
 *
 *
 */
