#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


void KalmanFilter::SetState(const VectorXd& state_mean,
                            const MatrixXd& state_cov) {
  state_mean_ = state_mean;
  state_cov_ = state_cov;
}

void KalmanFilter::Predict(const MatrixXd &F, const MatrixXd &Q) {
  state_mean_ = F * state_mean_;
  state_cov_ = F * state_cov_ * F.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd &y,
                          const MatrixXd &H,
                          const MatrixXd &R) {
  const MatrixXd S = (H * state_cov_ * H.transpose() + R);
  const MatrixXd K = (state_cov_ * H.transpose() * S.inverse());
  state_mean_ += K * y;
  state_cov_ -= (K * H * state_cov_).eval();
}

const Eigen::VectorXd& KalmanFilter::GetStateMean() const {
  return state_mean_;
}


const Eigen::MatrixXd& KalmanFilter::GetStateCov() const {
  return state_cov_;
}
