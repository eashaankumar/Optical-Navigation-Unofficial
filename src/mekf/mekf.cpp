#include "mekf.hpp"
#include "eigenmvn.hpp"
#include <Eigen/Dense>
#include <Eigen/QR>
#include <iostream>
#include <math.h>

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

void runMEKF(const ColVector3d& w_ref, const ColVector4d& q_ref, const double& dt, const ColVector4d& pred_meas,
             const Matrix6d& P, ColVector4d& att_err_est, const double& noise, ColVector4d& q_state) {
    // Note the quaternion convention used here is the 4th entry is the scalar

    // Quaternion Error q_err = q1xq2^-1

    //TODO (psalazar): Eignen-ize . . .
    q2(:,1:3)  = -q2(:,1:3);
    q_err(:,1) = q1(:,4).*q2(:,1)+q1(:,3).*q2(:,2)-q1(:,2).*q2(:,3)+q1(:,1).*q2(:,4);
    q_err(:,2) = -q1(:,3).*q2(:,1)+q1(:,4).*q2(:,2)+q1(:,1).*q2(:,3)+q1(:,2).*q2(:,4);
    q_err(:,3) = q1(:,2).*q2(:,1)-q1(:,1).*q2(:,2)+q1(:,4).*q2(:,3)+q1(:,3).*q2(:,4);
    q_err(:,4) = -q1(:,1).*q2(:,1)-q1(:,2).*q2(:,2)-q1(:,3).*q2(:,3)+q1(:,4).*q2(:,4);

    // Normalize Quaternion
    nn=(qm(:,1).^2+qm(:,2).^2+qm(:,3).^2+qm(:,4).^2).^0.5;
    qm(:,1)=qm(:,1)./nn;
    qm(:,2)=qm(:,2)./nn;
    qm(:,3)=qm(:,3)./nn;
    qm(:,4)=qm(:,4)./nn;

    // Quaternion Product
    A = [p(4)*q(1:3) + q(4)*p(1:3) - crs(p(1:3))*q(1:3);
         p(4)*q(4) - dot(p(1:3),q(1:3))];


    // Kinematics - using the quaternion product to propagate the attitude
         // Calculate covariance 'a'
         //

    // Measurement - using observed Earth-Moon angle to derive a measurement
         // Get Meas
         // Sensitivity and Transform to Correct/Body Frame
         // Update estimated attitude error

    // Update
         // Update P
         // Update Omega

}
