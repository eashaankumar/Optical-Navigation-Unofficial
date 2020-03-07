#include "controller.hpp"
#include <iostream>
#include <fstream> // For file copying
#include <string>
#include <sstream>
#include <iomanip>
#include <Eigen/Dense>


using namespace opnav_controller;
using namespace std;

using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;

// read ephemiris table
MatrixXd readCSV(string file, int rows, int cols)
{
  ifstream in(file);
  string line;
  int row = 0;
  int col = 0;
  MatrixXd output = MatrixXd(rows, cols);
  if (in.is_open())
  {
    while (getline(in, line) && row < rows)
    {
      char *ptr = (char *)line.c_str();
      int len = line.length();
      col = 0;
      char *start = ptr;
      for (int i = 0; i < len; i++)
      {
        if (ptr[i] == ',')
        {
          output(row, col++) = atof(start);
          start = ptr + i + 1;
        }
      }
      output(row, col) = atof(start);
      row++;
    }
    in.close();
  }
  else{
    std::cout << "File : " << file << " did not open" << std::endl;
  }
  return output;
}


// copy in binary mode
bool copyFile(const char *SRC, const char* DEST)
{
    std::ifstream src(SRC, std::ios::binary);
    std::ofstream dest(DEST, std::ios::binary);
    dest << src.rdbuf();
    return src && dest;
}

int main(int argc, char** argv )
{
    opnav_controller::ColVector6d state_estimate;
    state_estimate << -23315.3423, -46739.04576, -2797.180321, -0.395099037, -3.530117736, -0.855512484;
    int ITER = 132;
    int hour = 21;
    int day = 27;
    int month = 6;
    // Load ephemiris
    Matrix<double, Dynamic, 6> moonEph;
    Matrix<double, Dynamic, 6> sunEph;
    moonEph = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/Cesium_moon_eph.csv", ITER, 6);
    sunEph = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/Cesium_sun_eph.csv", ITER, 6);
    // 2020-06-27
    for(int i = 79; i <= ITER; i++){
        cout << "[OPNAV]: Iteration " << i << endl;
        stringstream ss_hour, ss_day, ss_month;
        // Hour
        ss_hour << std::setw(2) << std::setfill('0') << (hour % 24);
        hour = hour + 1;
        if (hour >= 24){
            hour = 0;
            day = day + 1;
        }
        ss_day << std::setw(2) << std::setfill('0') << day;
        if (day > 30){
            day = 1;
            month = 7;
        }
        // month
        ss_month << std::setw(2) << std::setfill('0') << month;

        system(("cp -rf /Users/eashaan/Downloads/IterwiseCesiumDataset/" + to_string(i) + "/Camera1 /Users/eashaan/github/Optical-Navigation/temp" + "  > /dev/null").c_str());
        system(("cp -rf /Users/eashaan/Downloads/IterwiseCesiumDataset/" + to_string(i) + "/Camera2 /Users/eashaan/github/Optical-Navigation/temp" + "  > /dev/null").c_str());
        system(("cp -rf /Users/eashaan/Downloads/IterwiseCesiumDataset/" + to_string(i) + "/Camera3 /Users/eashaan/github/Optical-Navigation/temp" + "  > /dev/null").c_str());

        // for(int camera = 1; camera <= 3; camera++){
        //     for(int offset = 0; offset < 8; offset++){
        //         string imagePath = "/Users/eashaan/Downloads/CesiumTraj/Camera" + to_string(camera) + "/" + to_string(offset) + "/cesium-canvas-2020-" + ss_month.str() + "-" + ss_day.str() + "T" + ss_hour.str() + "_08_03.021200000002863817Z.png";
        //         string dest = "/Users/eashaan/github/Optical-Navigation/temp/Camera" + to_string(camera) + "/-" + to_string(offset) + ".png";
        //         system(("cp " + imagePath + " " + dest + "  > /dev/null").c_str());
        //         // string newDataSetRoot = "/Users/eashaan/Downloads/IterwiseCesiumDataset/" + to_string(i) + "/Camera" + to_string(camera);
        //         // dest = newDataSetRoot + "/" + to_string(offset) + ".png";
        //         //system(("mkdir " + newDataSetRoot + "  > /dev/null").c_str());
        //         //system(("cp " + imagePath + " " + dest + "  > /dev/null").c_str());
        //         // system(("rm -v " + newDataSetRoot + "  > /dev/null").c_str());

        //     }
        // }
        opnav_controller::OPNAV_run(i-0, moonEph, sunEph, state_estimate);
        cout << "[OPNAV]: State Estimate: " << state_estimate << endl;
        system("rm -v /Users/eashaan/github/Optical-Navigation/temp/Camera1/*  > /dev/null");
        system("rm -v /Users/eashaan/github/Optical-Navigation/temp/Camera2/*  > /dev/null");
        system("rm -v /Users/eashaan/github/Optical-Navigation/temp/Camera3/*  > /dev/null");
    }
    //system("cp /Users/eashaan/Downloads/3600_sec_feb2019_oct19_image_to_traj/Camera1/0/Camera0001.png /Users/eashaan/github/Optical-Navigation/temp/Camera1");
    return 0;
}