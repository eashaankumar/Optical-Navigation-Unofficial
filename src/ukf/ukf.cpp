// #define _USE_MATH_DEFINES

#include "ukf.hpp"
#include "eigenmvn.hpp"
#include <Eigen/Dense>
#include <Eigen/QR>    
#include <iostream>
#include <math.h>

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

namespace ukf{
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::Dynamic;
using Eigen::LLT;
using Eigen::EigenMultivariateNormal;

int matlab_length(MatrixXd m){
    return std::max(m.rows(), m.cols());
}

void runUKF(RowVector6d moon_eph, RowVector6d sun_eph, ColVector6d orig_meas, int dt, ColVector6d &state_estimate, Matrix6d &P, Matrix6d &K){
    int iters = 1;
    // Choose Initial Covariance Matricies (These are all variances)
    
    // How wrong is our initial estimate of state? 
    // Units: (km^2)
    // ColVector6d temp;
    // temp << 100, 100, 100, 1e-5, 1e-6, 1e-5; // Initial Covariance Estimate
    // Matrix6d P = temp.matrix().asDiagonal();

    // How wrong our dynamics model is? e.g. how off in variance will we be due
    // to solar radiation pressure, galactic particles, and bad gravity model? 
    // Units: (km^2)
    ColVector6d temp;
    temp << 1, 1, 1, 1e-5, 1e-6, 1e-5;
    Matrix6d Q = temp.matrix().asDiagonal(); // Process Error Covariance

    // How bad are our sensors? 
    // Units: (pixels^2)
    int delta = 1;   // Pixel Error
    temp << 1, 1, 1, 1, 1, 0.1;
    Matrix6d R = temp.matrix().asDiagonal() *delta;     // Measurement Error Covariance

    // Filter Constants by Hunter Adams vha3@cornell.edu
    // Set Tuning Parameters: 
    // 10e-4 < a < 1, b = 2 for Gaussian 
    double alpha = 10e-04;
    double beta = 2;

    double nx = matlab_length(P);
    double nv = matlab_length(Q);
    double k = 3-nx;
    double lambda = alpha*alpha*(nx+k)-nx; // tunable
    double constant = sqrt(nx+nv+lambda);
    double center_weight = lambda/(nx+nv+lambda);
    double other_weight = 1/(2*(nx+nv+lambda));
    // Initialize Matricies of Interest (for speed)

    ////////////////////////////////////////////////////////

    // int dt = 60; // time between measurements (seconds)
    const float camera_const = 3280/(62.2*M_PI/180); // camera constant: PixelWidth/FOV  [pixels/radians]

    // Cholesky requires matricies to be positive definite
    // Cholesky is matrix generalization of square root
    LLT<Matrix6d> lltOfP(P); // compute the Cholesky decomposition of A
    Matrix6d Sx = lltOfP.matrixU();; // L factor in decomposition
    LLT<Matrix6d> lltOfQ(Q);
    Matrix6d Sv = lltOfQ.matrixU();

    // Generate Sigma Points
    MatrixXd noise;
    MatrixXd sigmas;
    makeSigmas(state_estimate, Sx, Sv, nx, nv, constant, sigmas, noise);

    // Propogate Sigma Points
    MatrixXd prop_sigmas(sigmas.rows(), sigmas.cols());
    prop_sigmas.setZero();
    for(int j = 0; j < matlab_length(sigmas); j++){
        ColVector6d newState;
        dynamicsModel(sigmas.col(j)+noise.col(j), dt, moon_eph, sun_eph, newState);
        prop_sigmas.col(j) = newState;
        
    }

    // Compute Expected Sigma Measurements 
    // MatrixXd sigmaMeasurements(6, matlab_length(prop_sigmas));
    MatrixXd sigmaMeasurements(sigmas.rows(), sigmas.cols()); // [ek485]: Should be the same thing
    sigmaMeasurements.setZero();

    // Sigma measurements are the expected measurements calculated from
    // running the propagated sigma points through measurement model
    ColVector6d means;
    means.setZero();
    EigenMultivariateNormal<double> normX_solver(means, R);
    for(int j = 0; j < matlab_length(sigmas); j++){
        ColVector6d measurement;
        measModel(prop_sigmas.col(j) + normX_solver.samples(1), moon_eph.row(0), sun_eph.row(0), camera_const, measurement);
        // [ek485]: sigmas + noise is a column vector, not row vector. It should function the same way.
        sigmaMeasurements.col(j) = measurement; 
    }
    
    // a priori estimates
    ColVector6d x_mean;
    x_mean.setZero();
    ColVector6d z_mean;
    z_mean.setZero();
    getMeans(prop_sigmas, sigmaMeasurements, center_weight, other_weight, x_mean, z_mean);

    // Calculates the covariances as weighted sums
    Matrix6d Pxx, Pxz, Pzz;
    findCovariances(x_mean, z_mean, prop_sigmas, sigmaMeasurements, center_weight, other_weight, alpha, beta, R, Pxx, Pxz, Pzz);

    // A posteriori Estimates
    // TODO [ek485]: Is it ok to use the same nvm instance?
    newEstimate(x_mean, z_mean, Pxx, Pxz, Pzz, orig_meas + normX_solver.samples(1), R, state_estimate, P, K);
}

void G(Matrix<double, 1, 3> rec, Matrix<double, 1, 3> rem, Matrix<double, 1, 3> rcm, Matrix<double, 1, 3> rcs, Matrix<double, 1, 3> res, Matrix<double, 1, 3> &output){
    Matrix<double, 1, 3> earth_term = -1 * UE * rec/pow(rec.norm(),3);
    double b = sqrt(pow((rcm * rcm.transpose()).value(), 3));
    Matrix<double, 1, 3> moon_term = UM * ( (rem-rec)/b - rem/pow(rem.norm(),3) );
    b = sqrt(pow((rcs * rcs.transpose()).value(), 3));
    Matrix<double, 1, 3> sun_term = US * ( (res-rec)/b - res/pow(res.norm(),3) );
    output = earth_term + moon_term + sun_term;
}

void dynamicsModel(ColVector6d state, int dt, RowVector6d moon_eph, RowVector6d sun_eph, ColVector6d &newState){
    // Runge Kutta 4th Order, one timestep
    // Horizons ephemeris is in km, km/s
    // STK output is also in km
    moon_eph = moon_eph * 1000;
    sun_eph = sun_eph * 1000;
    Matrix<double, 1, 3> rec = state.topRows(3).transpose()*1000;
    Matrix<double, 1, 3> rec_dot = state.bottomRows(3).transpose()*1000;

    // r = sqrt(rec.row(0)*rec.row(0) + rec.row(1)*rec.row(1) + rec.row(2)*rec.row(2))

    Matrix<double, 1, 3> rem = moon_eph.leftCols(3);
    Matrix<double, 1, 3> res = sun_eph.leftCols(3);
    Matrix<double, 1, 3> rcm = rec - rem;
    Matrix<double, 1, 3> rcs = rec - res;

    // RK4
    // k1 = k2 = k3 = k4 = rec_dot
    // (1/6)*(k1 + 2*k2 + 2*k3 + k4) can be simplified
    Matrix<double, 1, 3> position = rec + rec_dot * dt;

    Matrix<double, 1, 3> n1, n2, n3, n4;
    G(position, rem, rcm, rcs, res, n1);
    G(position+dt*n1/2, rem, rcm, rcs, res, n2); //TODO [ek485]: Can t be double?
    G(position+dt*n2/2, rem, rcm, rcs, res, n3);
    G(position+n3*dt, rem, rcm, rcs, res, n4);

    Matrix<double, 1, 3> velocity = rec_dot + (1.0/6.0)*(n1 + 2*n2 + 2*n3 + n4)*dt;
    RowVector6d new_state_temp;
    new_state_temp << position, velocity;
    newState = new_state_temp.transpose() / 1000;
}

void newEstimate(ColVector6d x_mean, ColVector6d z_mean, Matrix6d Pxx, Matrix6d Pxz, Matrix6d Pzz, ColVector6d orig_meas, Matrix6d R, ColVector6d &x_new, Matrix6d &P_new, Matrix6d &K){
    K = Pxz*Pzz.completeOrthogonalDecomposition().pseudoInverse(); // TODO [ek485]: Just how bad is this?
    // K = 0; // To test dynamics Model

    x_new = x_mean + K*(orig_meas - z_mean);
    P_new = Pxx - K * R * K.transpose();
}

void findCovariances(ColVector6d x_mean, ColVector6d z_mean, MatrixXd sigmas, MatrixXd measurements, double center_weight, double other_weight, double alpha, double beta, Matrix6d R, Matrix6d &Pxx, Matrix6d &Pxz, Matrix6d &Pzz){
    // Calculating weighted sums of Covariances 
    double centerr_weight = center_weight + 1 - alpha * alpha + beta;
    ColVector6d xx = sigmas.col(0) - x_mean;
    ColVector6d zz = measurements.col(0) - z_mean;
    
    Pxx = centerr_weight * (xx * xx.transpose());
    Pxz = centerr_weight * (xx * zz.transpose());
    Pzz = centerr_weight * (zz * zz.transpose());

    MatrixXd xx2(sigmas.rows(), sigmas.cols()-1);
    xx2 = sigmas.rightCols(sigmas.cols()-1).colwise() - x_mean;
    MatrixXd zz2(measurements.rows(), measurements.cols()-1);
    zz2 = measurements.rightCols(measurements.cols()-1).colwise() - z_mean;

    for(int i = 1; i < matlab_length(xx2); i++){
        Pxx = other_weight * xx2.col(i) * xx2.transpose().row(i) + Pxx;
        Pxz = other_weight * xx2.col(i) * zz2.transpose().row(i) + Pxz;
        Pzz = other_weight * zz2.col(i) * zz2.transpose().row(i) + Pzz;
    }

    Pzz = Pzz + R;
}

void makeSigmas(ColVector6d x_estimate, Matrix6d Sx, Matrix6d Sv, double nx, 
                double nv, double constant, Matrix<double, Dynamic, 
                Dynamic> &sigmas, Matrix<double, Dynamic, Dynamic> &noise){
    noise = MatrixXd::Zero(6, (int)(2*(nx+nv)) + 1);
    sigmas = MatrixXd::Zero(6, (int)(2*(nx+nv)) + 1);
    sigmas.col(0) = x_estimate;
    // Offset sigma point positively in state (by chol(P))
    for(int i = 1; i<=nx; i++){
        sigmas.col(i) = x_estimate + constant*Sx.col(i-1);
        noise.col(i) = MatrixXd::Zero(6,1); // [ek485] TODO: Is this redundant?
    }
    // Offset sigma point negatively in state (by chol(P))
    for(int i = nx+1; i <= 2*nx; i++){
        sigmas.col(i) = x_estimate - constant*Sx.col(i-nx-1);
        noise.col(i) = MatrixXd::Zero(6,1); // [ek485] TODO: Is this redundant?
    }
    // Offset sigma point positively in dynamic noise (by chol(Q))
    for(int i = 2*nx+1; i <= 2*nx+nv; i++){
        sigmas.col(i) = x_estimate;
        noise.col(i) = constant*Sv.col(i-2*nx-1);
    }
    // Offset sigma point negatively in dynamic noise (by chol(Q))
    for(int i = 2*nx+nv+1; i <= 2*(nx+nv); i++){
        sigmas.col(i) = x_estimate;
        noise.col(i) = -constant*Sv.col(i-2*nx-nv-1);
    }
}

void getMeans(MatrixXd sigmas, MatrixXd measurements, double center_weight, double other_weight, ColVector6d &x_mean, ColVector6d &z_mean) {
    x_mean = center_weight * sigmas.col(0) + other_weight * sigmas.rightCols(sigmas.cols()-1).rowwise().sum();
    z_mean = center_weight * measurements.col(0) + other_weight * measurements.rightCols(measurements.cols()-1).rowwise().sum();
}

void measModel(ColVector6d trajectory, RowVector6d moonEph, RowVector6d sunEph, double factor, ColVector6d &measurements){
    
    /*std::cout << "trajectory " << trajectory << std::endl;
    std::cout << "moonEph " << moonEph << std::endl;
    std::cout << "sunEph " << sunEph << std::endl;
    std::cout << "factor " << factor << std::endl;*/
    
    // Assumes that trajectory is transposed
    double x = trajectory[0] * 1000; // Convert km to m
    double y = trajectory[1] * 1000;
    double z = trajectory[2] * 1000;
    double dmx = moonEph[0] * 1000; // Convert km to m
    double dmy = moonEph[1] * 1000;
    double dmz = moonEph[2] * 1000;
    double dsx = sunEph[0] * 1000; // Convert km to m
    double dsy = sunEph[1] * 1000;
    double dsz = sunEph[2] * 1000;

    // Distance from us to Earth
    double p_c2 = x*x + y*y + z*z;
    double p_c = sqrt(p_c2);
    // Distance from us to Moon
    double p_cm = sqrt((dmx-x)*(dmx-x) + (dmy-y)*(dmy-y) + (dmz-z)*(dmz-z));
    // Distance from us to Sun
    double p_cs = sqrt((dsx-x)*(dsx-x) + (dsy-y)*(dsy-y) + (dsz-z)*(dsz-z));

    // Use dot products to find separation angle
    double num1 = -x*dmx - y*dmy - z*dmz + p_c2;
    double num2 = -x*dsx - y*dsy - z*dsz + p_c2;
    double num3 = dmx*(dsx-x) + dmy*(dsy-y) + dsz*(dmz-z) - z*dmz - x*dsx - y*dsy + p_c2;

    // Pixel Separation Between Bodies
    double z1 = factor * acos(num1/(p_c*p_cm)) ; // E to M
    double z2 = factor * acos(num2/(p_c*p_cs)) ; // E to S
    double z3 = factor * acos(num3/(p_cm*p_cs)); // M to S

    // Pixel Diameter of Bodies
    double z4 = 2*factor * atan(EARTH_RADIUS/p_c) ;  // E
    double z5 = 2*factor * atan(MOON_RADIUS/p_cm);   // M
    double z6 = 2*factor * atan(SUN_RADIUS/p_cs);    // S
    
    measurements << z1, z2, z3, z4, z5, z6;
}




}
