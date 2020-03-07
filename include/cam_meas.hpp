#ifndef _CAM_MEAS
#define _CAM_MEAS

#include <Eigen/Dense>
#include <vector>

namespace opnav_cam_meas{
    using std::vector;
    using Eigen::Matrix;
    using Eigen::Dynamic;
    using Eigen::Vector2d;
    typedef Matrix<double, 6, 1> ColVector6d;
    typedef Matrix<double, 1, 3> RowVector3d;
    typedef Matrix<double, 6, Dynamic> Matrix6Dd;

    struct DetectedBodyProperties {
        vector<Vector2d> center;
        vector<double> radius;
        vector<double> index;
        int flag;
    };

    struct CameraParameters {
        double hFov;
        double vFov;
        double hPix;
        double vPix;
        double dcam12;
        double dcam23;
        double dcam13;
    };

    /**
     * [omega] angular velocity vector of spacecraft, 1x3 vector (rad/s)
     * [dt] time difference between consecutive photos, (s)
     * @return
     * [h] vector of measurements (z1...z6 as described by Hunter)
     * (every column is one full measurement,
     * number of columns depends on the number of times a full EMS set appears
     * in the photo set).
     * [ek485] TODO: We are discarding measurements where one or more bodies is missing.
     */ 
    void OPNAV_cameraMeasurements(RowVector3d omega, double dt, Matrix6Dd &h);
} // namespace opnav_cam_meas

#endif