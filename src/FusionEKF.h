#ifndef SRC_FUSIONEKF_H_
#define SRC_FUSIONEKF_H_

#include "measurement_package.h"
#include "kalman_filter.h"
#include "tools.h"
#include "Eigen/Dense"

#include <vector>
#include <string>
#include <fstream>

class FusionEKF {
 public:
  explicit FusionEKF(IKalmanFilter* kf);
  FusionEKF& operator=(const FusionEKF&) = delete;

  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

 private:
  IKalmanFilter* const kf_;

  // check whether the tracking toolbox was initiallized or not (first
  // measurement)
  bool is_initialized_{false};

  // previous timestamp
  std::int64_t previous_timestamp_{0};

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
};

#endif  // SRC_FUSIONEKF_H_
