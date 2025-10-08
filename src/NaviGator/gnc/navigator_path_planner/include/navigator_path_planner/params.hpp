#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <string>

namespace lqrrt_params
{
  constexpr double DT = 0.1;
  constexpr double GOAL_TOLERANCE = 0.5;
  constexpr double STEP_SIZE = 0.5;
  constexpr double GOAL_BIAS = 0.05;
  constexpr double OGRID_THRESHOLD = 90.0;
  constexpr double MAX_LINEAR_VELOCITY = 1.0;
  constexpr double MAX_ANGULAR_VELOCITY = 0.5;
  constexpr int MAX_ITERS = 10000;
  constexpr int MAX_TREE_SIZE = 2000;

  constexpr double X_MIN = -20.0;
  constexpr double X_MAX =  20.0;
  constexpr double Y_MIN = -20.0;
  constexpr double Y_MAX =  20.0;
  constexpr double YAW_MIN = -M_PI;
  constexpr double YAW_MAX =  M_PI;

  inline double randomUniform(double min, double max) {
    return min + (max - min) * (double(rand()) / RAND_MAX);
  }

  inline Eigen::Vector3d sampleRandomState() {
    Eigen::Vector3d s;
    s(0) = randomUniform(X_MIN, X_MAX);
    s(1) = randomUniform(Y_MIN, Y_MAX);
    s(2) = randomUniform(YAW_MIN, YAW_MAX);
    return s;
  }
}
