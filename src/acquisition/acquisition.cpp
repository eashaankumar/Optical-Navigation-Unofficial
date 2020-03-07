#include "acquisition.hpp"
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>


#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

namespace opnav_acquisition{

using std::thread;

double getAngularVelocity(){
    return 6;
}

void switchCamera(int cam){
    std::cout << "[OPNAV] Acquiring Camera [" << cam << "]..." << std::endl;
}

void captureImg(int cam){
}

void writeToDisk(int cam){
}

// There should be no I/O except capture and write
void sequential_frames(int cam){
    double currentAngle = 315; // degrees
    double delta = 315; // degrees
    double timeImg = 0.4; // seconds
    double timeWrite = 1; // seconds
    while (currentAngle >= 0){
        double omega = getAngularVelocity() * 180 / M_PI;
        double timeWait = delta / omega - timeImg - timeWrite;
        // Wait for satellite to get into position
        std::this_thread::sleep_for(std::chrono::milliseconds(int(timeWait * 1000)));
        captureImg(cam); // takes timeImg seconds
        writeToDisk(cam); // takes timeWrite seconds
        currentAngle = currentAngle - 45;
    }
}

void OPNAV_beginAcquisition(){
    switchCamera(0);
    thread acq_0 (sequential_frames, 0);
    acq_0.join();

    switchCamera(1);
    thread acq_1 (sequential_frames, 1);
    acq_1.join();

    switchCamera(2);
    thread acq_2 (sequential_frames, 2);
    acq_2.join();
}

} // namespace opnav_acquisition