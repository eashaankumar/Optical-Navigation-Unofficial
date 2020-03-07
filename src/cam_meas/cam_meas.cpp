#include "cam_meas.hpp"
#include "find.hpp"
#include <stdio.h>
#include <algorithm>    // std::sort
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>       /* fmod */

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

namespace opnav_cam_meas {
using std::vector;
using std::sort;
using cv::glob;
using cv::String;
using cv::Mat;
using cv::Vec3f;
using cv::imread;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Dynamic;
using Eigen::Vector2d;
typedef Matrix<double, 6, 1> ColVector6d;
typedef Matrix<double, 1, 3> RowVector3d;
typedef Matrix<double, 6, Dynamic> Matrix6Dd;

void loadProperties(Mat img, find::SpaceBody bodyType, int cam, int imageNum, DetectedBodyProperties &xValues){
    vector<Vec3f> circles;
    find::OPNAV_findHoughCircle(img, circles, bodyType);
    String name;
    switch (bodyType){
        case find::SpaceBody::FIND_SUN:
            name = "Sun";
        break;
        case find::SpaceBody::FIND_EARTH:
            name = "Earth";
        break;
        case find::SpaceBody::FIND_MOON:
            name = "Moon";
        break;
    }
    if (circles.empty()){
        //std::cout << name << ": Failed" << std::endl;
    }
    else{
        std::cout << name << ": Success" << std::endl;
        Vector2d center(circles[0][0], circles[0][1]);
        std::cout << circles[0][0] << " " << circles[0][1] << " " << circles[0][2] << std::endl;
        xValues.center.push_back(center);
        xValues.radius.push_back(circles[0][2]);
        xValues.index.push_back(imageNum);
        xValues.flag = cam;
    }
}

    /**
 * [earthValues], [moonValues], [sunValues] contain 
 * information about successful detections in available images
 * [smallest_num_detections] fewest time we see a body
 * [camera_parameters] are camera parameters for the 3 cameras
 * [omega] angular velocity vector of spacecraft, 1x3 vector (rad/s)
 * [dt] time difference between consecutive photos, (s)
 * @return
 * [h] 6 x n matrix of measured values [z1 ... z6]
 * z1,z2,z3 = angular distance between EM, ES, and MS (pixels)
 * z4,z5,z6 = apparent diameter of E,M,S (pixels)
 */ 
void computeMeasurement(DetectedBodyProperties earthValues, 
                        DetectedBodyProperties moonValues, 
                        DetectedBodyProperties sunValues, 
                        int smallest_number_of_detections, 
                        CameraParameters cam_p, 
                        RowVector3d omega, 
                        double dt,
                        Matrix6Dd &h){

    h.resize(6,smallest_number_of_detections);
    for(int i = 0; i < smallest_number_of_detections; i++){
        // Time Difference between snaps of bodies
        double dtEM = abs(earthValues.index[i] - moonValues.index[i])*dt; 
        double dtES = abs(earthValues.index[i] - sunValues.index[i])*dt;
        double dtMS = abs(moonValues.index[i] - sunValues.index[i])*dt;
        // Angle Between Bodies (Convert to Degrees)
        double angEM = fmod(dtEM * omega[2], M_PI) * 180/M_PI;
        double angES = fmod(dtES * omega[2], M_PI) * 180/M_PI;
        double angMS = fmod(dtMS * omega[2], M_PI) * 180/M_PI;
        Matrix<double, 1, 2> one_zero(1, 0);
        Matrix<double, 1, 2> zero_one(0, 1);
        // Additional Horizontal Pixels Due to Time Offset
        Matrix<double, 1, 2> horiz_pixelsEM = angEM/cam_p.hFov * cam_p.hPix * one_zero;
        Matrix<double, 1, 2> horiz_pixelsES = angES/cam_p.hFov * cam_p.hPix * one_zero;
        Matrix<double, 1, 2> horiz_pixelsMS = angMS/cam_p.hFov * cam_p.hPix * one_zero;
        // Additional Vertical Pixels Due to Camera Position offset
        double vert_pixels12 = cam_p.dcam12/cam_p.vFov * cam_p.vPix ;
        double vert_pixels13 = cam_p.dcam13/cam_p.vFov * cam_p.vPix ;
        double vert_pixels23 = cam_p.dcam23/cam_p.vFov * cam_p.vPix ;
        Matrix3d vertPix;
        vertPix << 0,            vert_pixels12,    vert_pixels13,
                vert_pixels12,        0,            vert_pixels23, 
                vert_pixels13,    vert_pixels23,          0;
        // Measurements
        double z1 = ((earthValues.center[i].transpose() - moonValues.center[i].transpose()).cwiseAbs() + horiz_pixelsEM + vertPix.row(earthValues.flag).col(moonValues.flag)*zero_one).norm(); //EM
        double z2 = ((earthValues.center[i].transpose() - sunValues.center[i].transpose()).cwiseAbs() + horiz_pixelsEM + vertPix.row(earthValues.flag).col(sunValues.flag)*zero_one).norm(); //ES
        // [ek485] TODO: Is it vertPix[moon, sun] or vertPix[sun,moon]
        double z3 = ((sunValues.center[i].transpose() - moonValues.center[i].transpose()).cwiseAbs() + horiz_pixelsEM + vertPix.row(moonValues.flag).col(sunValues.flag)*zero_one).norm(); //SM
        double z4 = 2*earthValues.radius[i]; // E        
        double z5 = 2*moonValues.radius[i]; // M
        double z6 = 2*sunValues.radius[i]; // S
        Matrix<double, 6, 1> temp;
        temp << z1, z2, z3, z4, z5, z6;
        h.col(i) = temp;
    }
}

void OPNAV_cameraMeasurements(RowVector3d omega, double dt, Matrix6Dd &h){
    CameraParameters camera_parameters = {};
    // Camera Constants
    // Horizontal/Vertical Field of View (Degrees), Pixel Dimensions
    camera_parameters.hFov = 62.2;
    camera_parameters.vFov = 48.8;
    camera_parameters.hPix = 1685;
    camera_parameters.vPix = 813;
    // Angular Separation Between Cameras (degrees)
    /*camera_parameters.dcam12 = 40 ;
    camera_parameters.dcam13 = -40 ;
    camera_parameters.dcam23 = -80;*/
    camera_parameters.dcam12 = 40 ;
    camera_parameters.dcam13 = -40 ;
    camera_parameters.dcam23 = -80;

    DetectedBodyProperties earthValues = {};
    DetectedBodyProperties moonValues = {};
    DetectedBodyProperties sunValues = {};

    vector<String> cameraLocations;
    cameraLocations.push_back("/Users/eashaan/github/Optical-Navigation/temp/Camera1");
    cameraLocations.push_back("/Users/eashaan/github/Optical-Navigation/temp/Camera2");
    cameraLocations.push_back("/Users/eashaan/github/Optical-Navigation/temp/Camera3");

    // Check for celestial bodies for each camera
    for (size_t cam = 0; cam < cameraLocations.size(); cam ++){
        vector<String> files;
        glob(cameraLocations[cam], files, false);
        // vector<Mat> images;
        size_t len = files.size(); //number of png files in images folder
        for (size_t i = 0; i < len; i++){
            Mat img = imread(files[i]);
            // Find Earth
            if (earthValues.index.empty()){
                // if(cam > 0){
                //     printf("No Earth Found in Camera %zu, Looking in Camera %zu\n", cam - 1, cam);
                // }
                loadProperties(img, find::FIND_EARTH, cam, i, earthValues);
            }
            // Find Moon
            if (moonValues.index.empty()){
                // if(cam > 0){
                //     printf("No Moon Found in Camera %zu, Looking in Camera %zu\n", cam - 1, cam);
                // }
                loadProperties(img, find::FIND_MOON, cam, i, moonValues);
            }
            // Find Sun
            if (sunValues.index.empty()){
                // if(cam > 0){
                //     printf("No Sun Found in Camera %zu, Looking in Camera %zu\n", cam - 1, cam);
                // }
                loadProperties(img, find::FIND_SUN, cam, i, sunValues);
            }
        }

    }

    // Display warning if no set found 
    if (earthValues.index.empty()){
        printf("No Earth Found\n");
    }
    if (moonValues.index.empty()){
        printf("No Moon Found\n");
    }
    if (sunValues.index.empty()){
        printf("No Sun Found\n");
    }

    // Post Processing Images into Pixel Measurements 
    // Number of Times Each Body Was Identified
    // TODO (ek485): Code guarantees only one instance of each body is found
    size_t e = earthValues.index.size();
    size_t m = moonValues.index.size();
    size_t s = sunValues.index.size();
    vector<size_t> sorted{e, m, s};
    sort(sorted.begin(), sorted.end());
    // Casting will not be an issue since we are dealing with small numbers
    int smallest_number_of_detections = static_cast<int>(sorted[0]);
    computeMeasurement(earthValues, moonValues, sunValues, smallest_number_of_detections, camera_parameters, omega, dt, h);
}

// double piMod(double a) {
//     double b = fmod(a, M_PI);
//     return b;
// }
} // namespace opnav_cam_meas
