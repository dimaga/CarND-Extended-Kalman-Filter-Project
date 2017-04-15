#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cassert>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::endl;
using std::cout;

FusionEKF::FusionEKF(IKalmanFilter* kf)
: kf_(kf) {
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    using Eigen::Vector4d;
    using Eigen::Matrix4d;

    Vector4d state_mean{Vector4d::Zero()};
    Matrix4d state_cov{Matrix4d::Identity() * 10000};

    if (MeasurementPackage::RADAR == measurement_pack.sensor_type_) {
      const double ro = measurement_pack.raw_measurements_[0];
      const double phi = measurement_pack.raw_measurements_[1];
      const double ro_dot = measurement_pack.raw_measurements_[2];

      state_mean(0) = ro * std::cos(phi);
      state_mean(1) = ro * std::sin(phi);
      state_cov(0, 0) = state_cov(1, 1) = R_radar_(0, 0);

      if (std::abs(ro) > 1e-3) {
        state_mean.tail(2) = state_mean.head(2).normalized() * ro_dot;
        state_cov(2, 2) = state_cov(3, 3) = 100;
      }
    } else if (MeasurementPackage::LASER == measurement_pack.sensor_type_) {
      state_mean.head(2) = measurement_pack.raw_measurements_;
      state_cov.topLeftCorner<2, 2>() = R_laser_;
    } else {
      assert(false);
    }

    kf_->SetState(state_mean, state_cov);

    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  const double dts = 1e-6 * (measurement_pack.timestamp_ - previous_timestamp_);
  previous_timestamp_ = measurement_pack.timestamp_;

  MatrixXd F = Eigen::Matrix4d::Identity();
  F(1, 3) = F(0, 2) = dts;

  const double kNoiseAx = 9.0;
  const double kNoiseAy = 9.0;

  MatrixXd G(4, 2);
  G <<
    dts * dts * 0.5, 0.0,
    0.0, dts * dts * 0.5,
    dts, 0.0,
    0.0, dts;

  MatrixXd Qv(2, 2);
  Qv <<
    kNoiseAx, 0.0,
    0.0, kNoiseAy;

  const MatrixXd Q = G * Qv * G.transpose();

  kf_->Predict(F, Q);

  if (MeasurementPackage::RADAR == measurement_pack.sensor_type_) {
    Eigen::Vector3d predicted_z;

    const auto& state_mean = kf_->GetStateMean();
    if (PredictRadarMeasurement(state_mean, &predicted_z)) {
      MatrixXd H(3, 4);

      if (PredictRadarMeasurementJac(state_mean, &H)) {
        VectorXd y = measurement_pack.raw_measurements_ - predicted_z;

        while (y[1] > M_PI) {
          y[1] -= 2 * M_PI;
        }

        while (y[1] < -M_PI) {
          y[1] += 2 * M_PI;
        }

        kf_->Update(y, H, R_radar_);
      }
    }

  } else if (MeasurementPackage::LASER == measurement_pack.sensor_type_) {
    MatrixXd H(2, 4);
    H <<
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;

    const auto& state_mean = kf_->GetStateMean();
    const VectorXd y = measurement_pack.raw_measurements_ - H * state_mean;
    kf_->Update(y, H, R_laser_);
  } else {
    assert(false);
  }
}
