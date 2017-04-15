#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


void KalmanFilter::SetState(const VectorXd& state_mean,
                            const MatrixXd& state_cov) {
  x_ = state_mean;
  P_ = state_cov;
}

void KalmanFilter::Predict(const MatrixXd &F, const MatrixXd &Q) {
}

void KalmanFilter::Update(const VectorXd &y,
                          const MatrixXd &H,
                          const MatrixXd &R) {
}

const Eigen::VectorXd& KalmanFilter::GetStateMean() const {
  return x_;
}


const Eigen::MatrixXd& KalmanFilter::GetStateCov() const {
  return P_;
}
