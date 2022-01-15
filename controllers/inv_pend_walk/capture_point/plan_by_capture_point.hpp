#ifndef PLAN_BY_CAPTURE_POINT
#define PLAN_BY_CAPTURE_POINT
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <deque>
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
 * @brief capture pointによる歩行パターン生成方法
 *
 */
void footStepPlannerCapturePoint()
{
    double x = 0, y = 0, xinit = 0, yinit = 0;     //(m) CoMのワールド座標 {CoM = Center of Mass}
    double xd = 0, yd = 0, xdinit = 0, ydinit = 0; // CoMの速度 v(m/s) xdot(t)
    double px = 0.0, py = 0.0;               //(m)　着地位置のワールド座標 これは実用的にはローカルの方が良いのでは？？
    constexpr double Tc = std::sqrt(zh / g); //微分方程式の時定数
    constexpr double w = std::sqrt(g / zh);
    // const double Tsup = support_time - std::fmod(support_time, dt); //誤差を無くすため
    const double Tsup = 0.34;
    int_fast64_t step_n = 0;
    const int32_t steps_ = 3;
    const double stride_x = 0.2;
    const double stride_y = 0.8;
    double cp_x_target = 0.0, cp_y_target = 0.0;
    double cp_x_now = 0.0, cp_y_now = 0.0;
    double cp_x_old = 0.0, cp_y_old = 0.0;
    debugPrint(Tsup, "Tsup");
    std::deque<std::deque<double>> result;
    std::ofstream velofs("velo.dat");
    std::vector<FootPrint> footprint_list(100);
    for (double t = 0.0; t < (Tsup * static_cast<double>(steps_ * 2 + 1)); ++step_n)
    {
        //決められた次の一歩を着く地点までの遊脚の移動を行っている時のシミュレーション---------------
        for (double t_tmp = 0.0; t_tmp < Tsup; t_tmp += dt, t += dt)
        {
            double C = std::cosh(t_tmp / Tc);
            double S = std::sinh(t_tmp / Tc);
            double ewt = std::exp(w*t_tmp);
            double dT = Tsup - t_tmp;//次のCPまでの時間
            x = (xinit - px) * C + Tc * xdinit * S + px; // x,xdともにn歩目開始時の状態
            y = (yinit - py) * C + Tc * ydinit * S + py;
            xd = (xinit - px) / Tc * S + xdinit * C;
            yd = (yinit - py) / Tc * S + ydinit * C;
            // cp_x_now = ewt * cp_x_old + (1.0 - ewt)*px; //ここでのcp_targetは1ループ前の話。
            // cp_y_now = ewt * cp_y_old + (1.0 - ewt)*py;
            cp_x_now = x + xd / w;
            cp_y_now = y + yd / w;
            // xd = -w*(x - cp_x_now);
            // yd = -w*(y - cp_y_now);
            result.push_back({x, y});
            velofs << t << " " << xd << " " << yd << std::endl;
        }
        xinit = x;
        yinit = y;
        xdinit = xd;
        ydinit = yd;
        //次の一歩の目標位置を計算------------------
        double b  = std::exp(w*Tsup);
        std::cout << b << std::endl;
        double sx = 0.0, sy = 0.0; 
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
        cp_x_old = cp_x_target;
        cp_y_old = cp_y_target;
        cp_x_target = px + sx;
        cp_y_target = py + sy;
        px = 1.0 /(1.0 - b) * cp_x_target - b / (1.0 - b) * cp_x_now;  
        py = 1.0 /(1.0 - b) * cp_y_target - b / (1.0 - b) * cp_y_now;  

        static bool is_right = true;
        footprint_list.push_back({Tsup, px - xinit, py - yinit, is_right});
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
    // return footprint_list;
}

#endif // !PLAN_BY_CAPTURE_POINT