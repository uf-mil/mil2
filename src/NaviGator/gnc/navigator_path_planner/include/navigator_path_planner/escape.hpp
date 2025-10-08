#pragma once
#include <Eigen/Dense>

class EscapeBehavior {
public:
  Eigen::VectorXd escape(const Eigen::VectorXd &x);
};
