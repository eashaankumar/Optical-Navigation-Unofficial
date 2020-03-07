#include "cam_meas.hpp"
#include <iostream>

#include <Eigen/Dense>
#include <math.h>       /* fmod */

using namespace std;

double piMod(double a) {
    double b = fmod(a, 2*M_PI);
    return b;
}


int main(int argc, char** argv )
{
    printf("Main\n");
    
    // Eigen::Matrix3f m;
    // m << 1, -2.0, 3,
    //  4, 5, 6,
    //  7, 8, 9;
    // cout << m.row(0).col(1);
    
    // Matrix<double, 1, 2> one_zero(1, 0);
    // Matrix<double, 1, 2> a = 2 * one_zero;
    // Matrix<double, 1, 2> b = 3 * one_zero;

    // cout << a << "\n\n" << b << endl;

    Eigen::Matrix<double, 6, 3> m;
    cout << m << "\n" << endl;
    for (int i = 0; i < 3; i++){
        Eigen::Matrix<double, 6, 1> temp;
        temp << 1, 2, 3, 4, 5, 6;
        m.col(i) = temp;
    }
    cout << m << "\n" << endl;
    return 0;
}