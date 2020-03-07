# Optical-Navigation

C++ Implementation of the OpNav algorithms.

To build this project, run the following commands from the root project directory.

`cd build`

`cmake ..`

**Note**: cmake should print out the correct paths to `EIGEN` and `OPENCV`.

To build all executables, run:

`make`

## Find

This component will utilize Circle Hough Transform to identify celestial bodies.

`make OPNav_find_run`

`./OPNav_find_run <path to image>` to visualize the algorithm.

## UKF

The ukf module takes in the previous state and calculates the next state.

`make OPNav_ukf_run`

`./OPNav_ukf_run` to try UKF with a hard-coded example.

## Tests

We are using Google Test for this project. Setup google test by running the following commands from root project directory.

`mkdir lib`

`cd lib`

`git clone https://github.com/google/googletest.git`

`cd ..`

`cd build`

`cmake ..`

Build the test executable:

`make OPNav_test_run`

Run all tests:

`./OPNav_test_run`

Run a specific test named as METHOD.TESTNAME:

`./OPNav_test_run --gtest_filter=METHOD.TESTNAME`.

## Cleaning

`make clean`
