#include "tools.h"
#include <iostream>
#include <vector>
#include <cassert>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth) {
  assert(!estimations.empty());
  assert(estimations.size() == ground_truth.size());

  VectorXd resSq(estimations.front().size());
  resSq.setZero();

  for (std::size_t i = 0; i < estimations.size(); ++i) {
    const auto& est = estimations.at(i);
    const auto& gt = ground_truth.at(i);
    const auto residual = (est - gt).array();
    resSq.array() += residual * residual;
  }

  resSq /= estimations.size();

  return resSq.cwiseSqrt();
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {
  MatrixXd result(x_state.size(), x_state.size());
  result.setZero();
  /**
   TODO:
   * Calculate a Jacobian here.
   */
  return result;
}
