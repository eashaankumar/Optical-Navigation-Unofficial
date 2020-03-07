#ifndef _UKF
#define _UKF
#include <Eigen/Dense>

#define EARTH_RADIUS 6378000
#define MOON_RADIUS 1737000
#define SUN_RADIUS 695505000

#define UE 3.986e14
#define UM 4.904e12
#define US 1.327e20


namespace ukf{
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Dynamic;
typedef Matrix<double, 6, 1> ColVector6d;
typedef Matrix<double, 1, 6> RowVector6d;
typedef Matrix<double, 6, 6> Matrix6d;

/**
 * Returns size of largest dimension of [m]
 * Behaves like Matlab's length()
 */ 
int matlab_length(MatrixXd m);

/**
 * [moon_eph] 1x6
 * [sun_eph] 1x6
 * [measurements] 6x1
 * [init_estimate] 6x1
 * @returns
 * [P_store]
 * [state_estimate]
 * [K]
 */ 
void runUKF(RowVector6d moon_eph, RowVector6d sun_eph, ColVector6d orig_meas, int dt, ColVector6d &state_estimate, Matrix6d &P, Matrix6d &K);

void dynamicsModel(ColVector6d state, int dt, RowVector6d moon_eph, RowVector6d sun_eph, ColVector6d &newState);

void newEstimate(ColVector6d x_mean, ColVector6d z_mean, Matrix6d Pxx, Matrix6d Pxz, Matrix6d Pzz, ColVector6d orig_meas, Matrix6d R, ColVector6d &x_new, Matrix6d &P_new, Matrix6d &K);

/**
 * [x_mean] 6x1
 * [z_mean] 6x1
 * [sigmas] 6x6
 * [measurements] 6x6
 * [center_weight] double
 * [other_weight] double
 * [alpha] double
 * [beta] double
 * [R] 6x6
 * @returns [Pxx], [Pxz], [Pzz] all 6x6
 */ 
void findCovariances(ColVector6d x_mean, ColVector6d z_mean, MatrixXd sigmas, MatrixXd measurements, double center_weight, double other_weight, double alpha, double beta, Matrix6d R, Matrix6d &Pxx, Matrix6d &Pxz, Matrix6d &Pzz);

/**
 * [init_estimate] 6x1
 * [Sx] 6x6, chol(P)
 * [Sv] 6x6, chol(Q)
 * [nx] matlab_length(P)
 * [nv] matlab_length(Q)
 * [constant] constant
 * @returns sigmas, noise both of size 6x(2*(nx+nv)+1)
 */ 
void makeSigmas(ColVector6d init_estimate, Matrix6d Sx, Matrix6d Sv, double nx, double nv, double constant, Matrix<double, Dynamic, Dynamic> &sigmas, Matrix<double, Dynamic, Dynamic> &noise);

/**
 * [trajectory] 6x1
 * [moonEph] 1x6
 * [sunEph] 1x6
 * @returns measurements 6x1
 */ 
void measModel(ColVector6d trajectory, RowVector6d moonEph, RowVector6d sunEph, double factor, ColVector6d &measurements);

/**
 * [state]: ?
 * [t] double
 * [dt] double
 * [moonEph] 6x1
 * [sunEph] 6x1
 * @returns newState 6x1
 */ 
void dynamicsModel(MatrixXd state, double t, double dt, ColVector6d moonEph, ColVector6d sunEph, double factor, Matrix<double, Dynamic, Dynamic> &newState); 

/**
 * [sigmas]: 6x6?
 * [measurements] 6x1
 * @returns x_mean 6x1, z_mean 6x1
 */ 
void getMeans(MatrixXd sigmas, MatrixXd measurements, double center_weight, double other_weight, ColVector6d &x_mean, ColVector6d &z_mean);

}

#endif

