#include "../include/navigator_path_planner/lqrrt_planner.hpp"
#include <limits>
#include <cmath>

LqrrtPlanner::LqrrtPlanner() {
  tree_.reserve(lqrrt_params::MAX_TREE_SIZE);
}

std::vector<Eigen::VectorXd> LqrrtPlanner::plan(
    const Eigen::VectorXd &start, const Eigen::VectorXd &goal)
{
  tree_.clear();
  tree_.push_back({start, -1});

  for (int i = 0; i < lqrrt_params::MAX_ITERS; i++) {
    Eigen::VectorXd rand = (double(rand()) / RAND_MAX < lqrrt_params::GOAL_BIAS)
      ? goal
      : sampleRandom();

    int nearest_idx = nearest(rand);
    Eigen::VectorXd new_state = steer(tree_[nearest_idx].state, rand);

    if (!isFeasible(new_state)) continue;

    tree_.push_back({new_state, nearest_idx});

    if ((new_state - goal).norm() < lqrrt_params::GOAL_TOLERANCE) {
      std::vector<Eigen::VectorXd> path;
      int idx = tree_.size() - 1;
      while (idx >= 0) {
        path.push_back(tree_[idx].state);
        idx = tree_[idx].parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
  }

  return {};
}

Eigen::VectorXd LqrrtPlanner::sampleRandom() {
  return lqrrt_params::sampleRandomState();
}

int LqrrtPlanner::nearest(const Eigen::VectorXd &pt) {
  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = 0;
  for (int i = 0; i < (int)tree_.size(); i++) {
    double d = (tree_[i].state - pt).squaredNorm();
    if (d < min_dist) { min_dist = d; nearest_idx = i; }
  }
  return nearest_idx;
}

Eigen::VectorXd LqrrtPlanner::steer(
    const Eigen::VectorXd &from, const Eigen::VectorXd &to)
{
  Eigen::VectorXd dir = (to - from).normalized();
  return from + lqrrt_params::STEP_SIZE * dir;
}

bool LqrrtPlanner::isFeasible(const Eigen::VectorXd &state) {
  // placeholder for map collision check
  return true;
}
