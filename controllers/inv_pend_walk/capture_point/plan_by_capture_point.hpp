#ifndef PLAN_BY_CAPTURE_POINT
#define PLAN_BY_CAPTURE_POINT
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifdef PLANNER_DEBUG
static constexpr bool planner_debug = true;
#else
static constexpr bool planner_debug = false;
#endif

template <class T>
static void debugPrint(T &&s, const std::string &label)
{
    if (planner_debug)
        std::cout << "Debug planner:" << label << "::" << std::forward<T>(s) << std::endl;
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

/**
 * @brief
 *
 */
void footStepPlannerCapturePoint()
{
    double x = 0, y = 0, xi = 0, yi = 0;     //(m) CoMのワールド座標 {CoM = Center of Mass}
    double xd = 0, yd = 0, xdi = 0, ydi = 0; // CoMの速度 v(m/s) xdot(t)
    double px = 0.0, py = 0.0;               //(m)　着地位置のワールド座標 これは実用的にはローカルの方が良いのでは？？
    constexpr double Tc = std::sqrt(zh / g); //微分方程式の時定数
    double C = 0.0;                          // cosh((t - t0) / Tc)
    double S = 0.0;                          // sinh((t - t0) / Tc)
    // const double Tsup = support_time - std::fmod(support_time, dt); //誤差を無くすため
    const double Tsup = support_time;
    int_fast64_t step_n = 0;
    debugPrint(Tsup, "Tsup");
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
        xd_target = (C + 1) / (Tc * S) * sx;
        yd_target = (C - 1) / (Tc * S) * sy;
        px = -a * (C - 1) / D * (x_target - C * xi - Tc * S * xdi) - b * S / (Tc * D) * (xd_target - S * xi / Tc - C * xdi);
        py = -a * (C - 1) / D * (y_target - C * yi - Tc * S * ydi) - b * S / (Tc * D) * (yd_target - S * yi / Tc - C * ydi);
        static bool is_right = true;
        footprint_list.push_back({Tsup, px - xi, py - yi, is_right});
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
}

#endif // !PLAN_BY_CAPTURE_POINT