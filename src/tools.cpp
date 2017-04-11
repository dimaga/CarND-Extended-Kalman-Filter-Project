#include "tools.h"
#include <iostream>
#include <vector>
#include <cassert>

using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using std::vector;

VectorXd EvaluateRmse(const vector<VectorXd> &estimations,
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

Vector3d PredictRadarMeasurement(const Vector4d& x_state) {
  Vector3d result;
  return result;
}

Matrix3d PredictRadarMeasurementJac(const Vector4d& x_state) {
  Matrix3d result;
  result.setZero();
  /**
   TODO:
   * Calculate a Jacobian here.
   */
  return result;
}
