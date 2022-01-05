#include "foot_step_planner.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
int main(int argc, char const *argv[])
{
    footStepPlanner(2.0,2.0,0.30);
    system("sleep 0.1");
    system("gnuplot-x11 -persist show.plt");
    return 0;
}
