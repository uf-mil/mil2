#pragma once
#include <vector>
#include <Eigen/Dense>
#include "params.hpp"

class LqrrtPlanner {
public:
  LqrrtPlanner();

  struct Node {
    Eigen::VectorXd state;
    int parent;
  };

  std::vector<Eigen::VectorXd> plan(
      const Eigen::VectorXd &start,
      const Eigen::VectorXd &goal);

private:
  std::vector<Node> tree_;
  Eigen::VectorXd sampleRandom();
  int nearest(const Eigen::VectorXd &pt);
  Eigen::VectorXd steer(const Eigen::VectorXd &from, const Eigen::VectorXd &to);
  bool isFeasible(const Eigen::VectorXd &state);
};
