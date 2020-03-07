#include "find.hpp"
#include <opencv2/opencv.hpp>
#include <vector> 

namespace find {
using ::std::make_tuple;
using ::std::vector;
using ::std::tuple;
using ::std::get;
using ::cv::Vec3f;
using ::cv::Scalar;
using ::cv::Size;
using ::cv::Mat;
using ::cv::COLOR_BGR2HSV;
using ::cv::COLOR_GRAY2BGR;
using ::cv::COLOR_BGR2GRAY;
using ::cv::HOUGH_GRADIENT;
using ::cv::inRange;
using ::cv::cvtColor;


int roundUpToOdd(float number){
    int n = (int)(ceil(number));
    return n % 2 == 0 ? n + 1 : n;
}

/**
 * Parameter Settings:
 * min distance between detected centers is set high
 * min and max radius is not specified. 
 * @precondition: If Sun in src, then 
 */ 
void OPNAV_findHoughCircle(Mat src, vector<Vec3f> &out_circles, SpaceBody body){
    vector<tuple<Scalar, Scalar>> boundaries;    
    switch (body)
    {
    case FIND_SUN:
        boundaries.push_back(make_tuple(Scalar(0,0,255), Scalar(180,51,255))); //HSV
        OPNAV_findHoughCircleHSV(src, out_circles, boundaries, 2,src.rows/50,80,15,1,0);

        // boundaries.push_back(make_tuple(Scalar(150, 150, 150), Scalar(255, 255, 255))); //RGB
        // OPNAV_findHoughCircleRGB(src, out_circles, boundaries, 2,1,50,10,0,0);
        break;
    case FIND_EARTH:
        boundaries.clear();
        boundaries.push_back(make_tuple(Scalar(80,30,0), Scalar(160,255,255))); //HSV
        OPNAV_findHoughCircleHSV(src, out_circles, boundaries, 2,50,80,30,10,0);

        // boundaries.push_back(make_tuple(Scalar(1, 1, 1), Scalar(255, 150, 205))); //RGB
        // OPNAV_findHoughCircleRGB(src, out_circles, boundaries, 2,50,80,30,10,0);
        break;
    case FIND_MOON:
        boundaries.clear();
        boundaries.push_back(make_tuple(Scalar(0, 0, 0), Scalar(179, 25, 255))); // HSV
        OPNAV_findHoughCircleHSV(src, out_circles, boundaries, 2,100,400,30,1,0);

        // boundaries.push_back(make_tuple(Scalar(90, 90, 90), Scalar(200, 200, 200))); //RGB
        // OPNAV_findHoughCircleRGB(src, out_circles, boundaries, 2,100,400,80,1,0);
        break;
    default:
        break;
    }
}

void OPNAV_findHoughCircleRGB(Mat src, vector<Vec3f> &out_circles, vector<tuple<Scalar, Scalar>> boundaries, double dp, double min_dist, double param1, double param2, int min_radius, int max_radius){
    // Mask
    Mat mask, output;

    //cvtColor(src, original_gray, COLOR_BGR2GRAY);
    for(int i = 0; i < boundaries.size(); i++){
        Scalar lower = get<0>(boundaries[i]);
        Scalar upper = get<1>(boundaries[i]);
        inRange(src,lower, upper, mask);
        bitwise_and(src, src, output, mask);
        // imshow("masked result (GRAY)", output);
        // cv::waitKey(0);
    }

    // // Hough Transform
    Mat gray, gray_blurred;
    cvtColor(output, gray, COLOR_BGR2GRAY);
    // cout << mask.rows << " / 50 = " << ksize << endl;
    int ksize = roundUpToOdd(src.rows / 50);
    medianBlur(gray, gray_blurred, ksize);
    // cv::imshow("median blur", gray_blurred);
    // cv::waitKey(0);

    HoughCircles(gray_blurred, out_circles, HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);
}

void OPNAV_findHoughCircleHSV(Mat src, vector<Vec3f> &out_circles, vector<tuple<Scalar, Scalar>> boundaries, double dp, double min_dist, double param1, double param2, int min_radius, int max_radius){
    // Mask
    int ksize = roundUpToOdd(0);
    Mat srcHSV, mask, output;

    cvtColor(src, srcHSV, COLOR_BGR2HSV);
    for(int i = 0; i < boundaries.size(); i++){
        Scalar lower = get<0>(boundaries[i]);
        Scalar upper = get<1>(boundaries[i]);
        inRange(srcHSV,lower, upper, mask);
        cvtColor(mask, srcHSV, COLOR_GRAY2BGR);
        bitwise_and(srcHSV, src, output);
        imshow("masked result (GRAY)", output);
        cv::waitKey(0);
    }

    // // Hough Transform
    Mat gray, gray_blurred;
    cvtColor(output, gray, COLOR_BGR2GRAY);
    // cout << mask.rows << " / 50 = " << ksize << endl;
    medianBlur(gray, gray_blurred, ksize);
    cv::imshow("median blur", gray_blurred);
    cv::waitKey(0);

    HoughCircles(gray, out_circles, HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);
}

} // namespace find