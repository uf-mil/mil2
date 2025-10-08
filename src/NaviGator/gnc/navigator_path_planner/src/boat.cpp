#include "../include/navigator_path_planner/vehicle_boat.hpp"

Eigen::VectorXd BoatDynamics::step(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
  Eigen::VectorXd x_next = x;
  x_next(0) += u(0) * std::cos(x(2)) * lqrrt_params::DT;
  x_next(1) += u(0) * std::sin(x(2)) * lqrrt_params::DT;
  x_next(2) += u(1) * lqrrt_params::DT;
  return x_next;
}
