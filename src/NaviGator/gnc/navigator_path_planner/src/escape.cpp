#include "../include/navigator_path_planner/vehicle_escape.hpp"
#include <cmath>

Eigen::VectorXd EscapeBehavior::escape(const Eigen::VectorXd &x) {
  Eigen::VectorXd x_next = x;
  x_next(2) += M_PI / 2.0;  // rotate in place
  return x_next;
}
