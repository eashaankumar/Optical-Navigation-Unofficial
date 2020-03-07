#include "controller.hpp"
#include "acquisition.hpp"
#include "cam_meas.hpp"
#include "find.hpp"
#include "ukf.hpp"
#include <unistd.h>
#include <iostream>
#include <Eigen/Dense>

namespace opnav_controller{

using Eigen::Dynamic;
using Eigen::Matrix;
using std::string;
using std::ifstream;
using ukf::ColVector6d;
using ukf::Matrix6d;
typedef Matrix<double, 1, 3> RowVector3d;
typedef Matrix<double, 6, Dynamic> Matrix6Dd;

// TODO [ek485]
void getOmegaVector(RowVector3d &omega){
    omega << 0, 0, 6;
}

void OPNAV_run(int currentTime, Matrix<double, Dynamic, 6> moonEph, Matrix<double, Dynamic, 6> sunEph, ColVector6d &state_estimate){
    std::cout << "[OPNAV] Starting..." << std::endl;

    opnav_acquisition::OPNAV_beginAcquisition();
    RowVector3d omega;
    getOmegaVector(omega);
    std::cout << "[OPNAV] Omega Vector: " << omega << std::endl;
    // TODO [ek485]: Get dt for each photo from acquisition module
    Matrix6Dd meas;
    // photos taken 45 deg apart, acquisition algorithm waits for 315 deg, (315 * PI/180) / 6 rad per sec
    opnav_cam_meas::OPNAV_cameraMeasurements(omega, 0.91, meas);
    if(meas.cols() > 0){
        const int STEPS = 132;
        ColVector6d temp;
        temp << 100, 100, 100, 1e-5, 1e-6, 1e-5; // Initial Covariance Estimate
        Matrix6d P = temp.matrix().asDiagonal();
        Matrix6d K;
        // TODO [ek485]: Figure out the correct dt (not 60)
        ukf::runUKF(moonEph.row(currentTime), sunEph.row(currentTime), meas, 60, state_estimate, P, K);
    }
    else{
        std::cout << "[OPNAV] Skipping ukf." << std::endl;
    }

    std::cout << "[OPNAV] Finished." << std::endl;
}

} // namespace opnav_controller