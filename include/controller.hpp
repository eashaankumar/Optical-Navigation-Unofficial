#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <Eigen/Dense>

namespace opnav_controller{
    using Eigen::Matrix;
    using Eigen::Dynamic;
    typedef Matrix<double, 6, 1> ColVector6d;

    void OPNAV_run(int currentTime, Matrix<double, Dynamic, 6> moonEph, Matrix<double, Dynamic, 6> sunEph, ColVector6d &state_estimate);
} // namespace opnav_controller

#endif