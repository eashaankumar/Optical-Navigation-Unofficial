#ifndef _FIND_H
#define _FIND_H
#include <opencv2/opencv.hpp>

namespace find{
// enum SpaceBody
enum SpaceBody {
    FIND_SUN,
    FIND_EARTH,
    FIND_MOON
};

using ::std::vector;
using ::std::tuple;
using ::cv::Vec3f;
using ::cv::Scalar;
using ::cv::Mat;

void OPNAV_findHoughCircleHSV(Mat src, vector<Vec3f> &out_circles, vector<tuple<Scalar, Scalar>> boundaries, double dp, double min_dist, double param1, double param2, int min_radius, int max_radius);
void OPNAV_findHoughCircleRGB(Mat src, vector<Vec3f> &out_circles, vector<tuple<Scalar, Scalar>> boundaries, double dp, double min_dist, double param1, double param2, int min_radius, int max_radius);

void OPNAV_findHoughCircle(Mat src, vector<Vec3f> &out_circles, SpaceBody body);

// TODO(ek485): Implement child function for segmentation
void OPNAV_findSegmentation(Mat src, vector<Vec3f> &out_circles, SpaceBody body);

} // namespace find

#endif