#pragma once
#include <Eigen/Dense>
#include "path_planner_cpp/params.hpp"

class BoatDynamics {
public:
  Eigen::VectorXd step(const Eigen::VectorXd &x, const Eigen::VectorXd &u);
};
