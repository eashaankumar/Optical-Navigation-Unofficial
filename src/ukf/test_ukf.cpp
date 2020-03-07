#include "ukf.hpp"
#include "eigenmvn.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace ukf;
using namespace Eigen;
using namespace std;

void testMeasModel(){
    ColVector6d traj;
    traj << 883.9567, 1.023e+03, 909.665, 65.648, 11.315, 28.420;
    ColVector6d moonEph;
    moonEph << 1.536e+05, -3.723e+05, 2.888e+03, 0.9089, 0.3486, -0.0880;
    ColVector6d sunEph;
    sunEph << -3.067e+07, -1.441e+08, 6.67e+03, 29.6329, -6.0859, -8.8015e-04;
    float factor = 1;

    ColVector6d measurements;
    measModel(traj, moonEph, sunEph, factor, measurements);

    cout << "Measurements " << measurements << endl;
}

void testUKF(){
    ColVector6d init_estimate;
    init_estimate << 2.331e+04, -4.673e+04, -2.795e+03, -0.395, -3.530, -0.856;

    Matrix<double, 1, 3> pos;
    pos << 1, 2, 3;
    Matrix<double, 1, 3> vel;
    vel << 4, 5, 6;
    RowVector6d fin;
    fin << pos, vel;
    ColVector6d flip;
    flip = fin.transpose();
    cout << "pos: " << pos << endl;
    cout << "vel: " << vel << endl;
    cout << "fin: " << fin << endl;
    cout << "flip: " << flip << endl;

}

int main(int argc, char** argv )
{
    testUKF();    
    return 0;
}