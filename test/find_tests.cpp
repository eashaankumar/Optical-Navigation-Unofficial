#include <gtest/gtest.h>
#include "find.hpp"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;
using namespace find;

const double CENTER_THRESHOLD = 10; // pixels
const double RADIUS_THRESHOLD = 10; // pixels

TEST(HoughCircle, EarthMoon1){
    const char* filename = "../EarthMoon.jpg";
    Vec3i moon(1449,995,355);
    Vec3i earth(305,440,50);
    Mat src = imread( samples::findFile( filename ), IMREAD_COLOR );
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", filename);
    }
    vector<Vec3f> sun_circles;
    vector<Vec3f> earth_circles;
    vector<Vec3f> moon_circles;

    OPNAV_findHoughCircle(src, sun_circles, FIND_SUN);
    OPNAV_findHoughCircle(src, earth_circles, FIND_EARTH);
    OPNAV_findHoughCircle(src, moon_circles, FIND_MOON);

    // Check accuracy of first detected circle for each body

    // There should be no Sun
    EXPECT_TRUE(sun_circles.size() == 0);
    EXPECT_TRUE(moon_circles.size() > 0);
    EXPECT_TRUE(earth_circles.size() > 0);

    // Moon
    Vec3i circle = moon_circles[0];
    double center_diff = sqrt(pow(moon[0] - circle[0], 2) + pow(moon[1] - circle[1], 2));
    double radius_diff = abs(circle[2] - moon[2]);
    EXPECT_TRUE(center_diff <= CENTER_THRESHOLD);
    EXPECT_TRUE(radius_diff <= RADIUS_THRESHOLD);

    // Earth
    circle = earth_circles[0];
    center_diff = sqrt(pow(earth[0] - circle[0], 2) + pow(earth[1] - circle[1], 2));
    radius_diff = abs(circle[2] - earth[2]);
    EXPECT_TRUE(center_diff <= CENTER_THRESHOLD);
    EXPECT_TRUE(radius_diff <= RADIUS_THRESHOLD);    
}

TEST(HoughCircle, EarthMoon2){
    const char* filename = "../EarthMoonfromMars.jpg";
    Vec3i moon(956,235,18);
    Vec3i earth(200,696,60);
    Mat src = imread( samples::findFile( filename ), IMREAD_COLOR );
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", filename);
    }
    vector<Vec3f> sun_circles;
    vector<Vec3f> earth_circles;
    vector<Vec3f> moon_circles;

    OPNAV_findHoughCircle(src, sun_circles, FIND_SUN);
    OPNAV_findHoughCircle(src, earth_circles, FIND_EARTH);
    OPNAV_findHoughCircle(src, moon_circles, FIND_MOON);

    // Check accuracy of first detected circle for each body

    // There should be no Sun
    EXPECT_TRUE(sun_circles.size() == 0);
    EXPECT_TRUE(moon_circles.size() > 0);
    EXPECT_TRUE(earth_circles.size() > 0);

    // Moon
    Vec3i circle = moon_circles[0];
    double center_diff = sqrt(pow(moon[0] - circle[0], 2) + pow(moon[1] - circle[1], 2));
    double radius_diff = abs(circle[2] - moon[2]);
    EXPECT_TRUE(center_diff <= CENTER_THRESHOLD);
    EXPECT_TRUE(radius_diff <= RADIUS_THRESHOLD);

    // Earth
    circle = earth_circles[0];
    center_diff = sqrt(pow(earth[0] - circle[0], 2) + pow(earth[1] - circle[1], 2));
    radius_diff = abs(circle[2] - earth[2]);
    EXPECT_TRUE(center_diff <= CENTER_THRESHOLD);
    EXPECT_TRUE(radius_diff <= RADIUS_THRESHOLD);    
}