#include <gtest/gtest.h>
#include "ukf.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

void handler(int sig)
{
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

using namespace ukf;
using namespace Eigen;
using namespace std;

string prefix(bool val)
{
  return val ? "\033[32m" : "\033[31m";
}

void AssertMatrixEquals(const MatrixXd &expected, const MatrixXd &actual, double equalityThreshold)
{
  int rows = expected.rows();
  int cols = expected.cols();

  //Assert that the matrices are the same size
  ASSERT_EQ(rows, actual.rows());
  ASSERT_EQ(cols, actual.cols());

  MatrixXd diff = (expected - actual);
  double maxDiff = diff.lpNorm<Infinity>();
  bool satisfied = maxDiff < equalityThreshold;

  if (satisfied)
    SUCCEED();
  else
  {
    cout << prefix(satisfied) << "Max single measurement difference (" << maxDiff << ") is greater than threshold (" << equalityThreshold << ")\n";
    FAIL();
  }
}

void AssertMatrixEquals(const MatrixXd &expected, const MatrixXd &actual)
{
  const double EQUALITY_THRESHOLD = 0.001;
  AssertMatrixEquals(expected, actual, EQUALITY_THRESHOLD);
}

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
      cout << ">>>>>>>Bruh: " << endl;
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
    cout << "File : " << file << " did not open" << endl;
  }
  return output;
}

const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

void writeCSV(string name, MatrixXd matrix)
{
  ofstream file(name.c_str());
  file << matrix.format(CSVFormat);
}

TEST(MeasModel, TestMeasurementOutputAccuracy)
{
  const int STEPS = 1000;
  float factor = 3280 / (62.2 * M_PI / 180);

  Matrix<double, STEPS, 6> traj;
  Matrix<double, STEPS, 6> expMeas;
  Matrix<double, STEPS, 6> calcMeas;
  Matrix<double, STEPS, 6> moonEph;
  Matrix<double, STEPS, 6> sunEph;

  moonEph = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c3_moon_eph", STEPS, 6);
  sunEph = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c3_sun_eph", STEPS, 6);
  traj = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c3_trajectory", STEPS, 6);
  expMeas = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c3_measurements", STEPS, 6);

  ColVector6d tmp;

  for (int i = 0; i < STEPS; i++)
  {
    measModel(traj.row(i).transpose(), moonEph.row(i), sunEph.row(i), factor, tmp);
    calcMeas.row(i) = tmp.transpose();
  }
  cout << expMeas << endl;
  cout << calcMeas << endl;
  AssertMatrixEquals(expMeas, calcMeas);
}

TEST(UKF, TestUKFOutputAccuracy)
{
  const int STEPS = 1;

  Matrix<double, STEPS + 1, 6> traj;
  Matrix<double, STEPS, 6> meas;
  Matrix<double, STEPS, 6> moonEph;
  Matrix<double, STEPS, 6> sunEph;
  Matrix<double, STEPS + 1, 6> matlabState;
  Matrix<double, STEPS, 6> simTraj;

  //read inputs
  traj = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c1_discretized_traj.csv", STEPS + 1, 6);
  meas = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c1_discretized_meas.csv", STEPS, 6);
  moonEph = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c1_discretized_moon_eph.csv", STEPS, 6);
  sunEph = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c1_discretized_sun_eph.csv", STEPS, 6);
  matlabState = readCSV("/Users/eashaan/github/Optical-Navigation/test/test_data/c1_discretized_ukf_k0.csv", STEPS + 1, 6);

  ColVector6d state_estimate = traj.row(0).transpose();
  ColVector6d temp;
  temp << 100, 100, 100, 1e-5, 1e-6, 1e-5; // Initial Covariance Estimate
  Matrix6d P = temp.matrix().asDiagonal();
  Matrix6d K;

  //For some reason the C3 data supplied has an error in the first
  //time step, so we'll start on the second (index of 1)
  for (int i = 0; i < STEPS; i++)
  {
    runUKF(moonEph.row(i), sunEph.row(i), meas.row(i).transpose(),60, state_estimate, P, K);
    simTraj.row(i) = state_estimate.transpose();
  }

  //cout << matlabState  << endl;
  cout << "Our state:       " << state_estimate.transpose() << endl;
  cout << "Matlab k=0 state: " << matlabState.row(STEPS) << endl;
  cout << "True state:       " << traj.row(STEPS) << endl;
  
  AssertMatrixEquals(state_estimate.transpose(),traj.row(STEPS),1000);
}
