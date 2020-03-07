cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(OPNav C CXX)
 
find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

message("EIGEN: ${EIGEN3_INCLUDE_DIR}")
message("OPENCV: ${OpenCV_INCLUDE_DIRS}")

include_directories( ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} include)

# Aquisition
set(ACQUISITION_BINARY ${CMAKE_PROJECT_NAME}_acquisition)
add_subdirectory(src/acquisition)

# Find
set(FIND_BINARY ${CMAKE_PROJECT_NAME}_find)
add_subdirectory(src/find)

# UKF
set(UKF_BINARY ${CMAKE_PROJECT_NAME}_ukf)
add_subdirectory(src/ukf)

# Camera Measurements
set(CAMMEAS_BINARY ${CMAKE_PROJECT_NAME}_cammeas)
add_subdirectory(src/cam_meas)

# Controller
set(CONTROLLER_BINARY ${CMAKE_PROJECT_NAME}_controller)
add_subdirectory(src/controller)

# Tests
enable_testing()
add_subdirectory(lib/googletest)
set(TEST_BINARY ${CMAKE_PROJECT_NAME}_test)
add_subdirectory(test)
