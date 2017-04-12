#include "tools.h"
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using std::vector;

namespace {
bool IsRadarMeasurementAllowed(const Vector4d& x_state) {
  return std::abs(x_state[0]) > 1e-5 || std::abs(x_state[1]) > 1e-5;
}
}

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

bool PredictRadarMeasurement(const Vector4d& x_state,
                             Vector3d* pMeasurement) {
  if (!IsRadarMeasurementAllowed(x_state)) {
    return false;
  }

  const double ro = x_state.head(2).norm();

  (*pMeasurement) <<
    ro,
    std::atan2(x_state[1], x_state[0]),
    x_state.head(2).dot(x_state.tail(2)) / ro;

  return true;
}

bool PredictRadarMeasurementJac(const Vector4d& x_state,
                                MatrixXd* pMeasurementJac) {
  if (!IsRadarMeasurementAllowed(x_state)) {
    return false;
  }

  return true;
}
