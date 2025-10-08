#include "../include/navigator_path_planner/vehicle_car.hpp"

Eigen::VectorXd CarDynamics::step(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
  Eigen::VectorXd x_next = x;
  x_next(0) += u(0) * std::cos(x(2)) * lqrrt_params::DT;
  x_next(1) += u(0) * std::sin(x(2)) * lqrrt_params::DT;
  x_next(2) += (u(0) / 2.0) * std::tan(u(1)) * lqrrt_params::DT;
  return x_next;
}
