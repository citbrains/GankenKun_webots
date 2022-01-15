#include <iostream>
#include <fstream>
#include "plan_by_capture_point.hpp"
using namespace Eigen;

int main(int argc, char const *argv[])
{
    footStepPlannerCapturePoint();
    system("sleep 0.1");
    system("gnuplot -persist show.plt");
    return 0;
}
