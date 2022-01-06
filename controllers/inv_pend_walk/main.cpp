#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

int main(int argc, char const *argv[])
{
    std::ofstream ofs("d.dat");
    ofs.close();
    system("sleep 0.1");
    system("gnuplot -persist show.plt");
    return 0;
}
