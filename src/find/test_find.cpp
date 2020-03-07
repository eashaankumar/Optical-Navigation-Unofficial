#include "find.hpp"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace cv;
using namespace std;
using namespace find;

int main(int argc, char** argv )
{
    // Mat A(30, 40, DataType<float>::type);
    const char* filename = argv[1];
    // Load image
    Mat src = imread( samples::findFile( filename ), IMREAD_COLOR );
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", filename);
    }

    vector<Vec3f> sun_circles;
    vector<Vec3f> earth_circles;
    vector<Vec3f> moon_circles;

    // OPNAV_findHoughCircle(src, sun_circles, FIND_SUN);
    // OPNAV_findHoughCircle(src, earth_circles, FIND_EARTH);
    OPNAV_findHoughCircle(src, moon_circles, FIND_MOON);

    // Display 
    // cout << "SUN" << endl; 
    // for (Vec3f a : sun_circles){
    //     cout << a << endl;
    // }
    // cout << "EARTH" << endl; 
    // for (Vec3f a : earth_circles){
    //     cout << a << endl;
    // }    
    cout << "MOON" << endl; 
    for (Vec3f a : moon_circles){
        cout << a << endl;
    }
    vector<Vec3f> circles;
    vector<Scalar> colors;
    // if(sun_circles.size() > 0){
    //     circles.push_back(sun_circles[0]);
    //     colors.push_back(Scalar(0, 255, 255));
    // }
    // if(earth_circles.size() > 0){
    //     circles.push_back(earth_circles[0]);
    //     colors.push_back(Scalar(255, 0, 0));
    // }    
    if(moon_circles.size() > 0){
        circles.push_back(moon_circles[0]);
        colors.push_back(Scalar(220, 220, 220));
    }
    for( size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        // circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        // if(i == 0){
        //     circle( src, center, radius, Scalar(0,255,0), 3, LINE_AA);
        // }
        circle( src, center, radius, colors[i], 3, LINE_AA);
        
        circle( src, center, 2, Scalar(0,0,255), 4, LINE_AA);
    }
    imshow("detected circles", src);
    waitKey();
    return 0;
}