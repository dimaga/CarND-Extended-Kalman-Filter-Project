#ifndef SRC_TOOLS_H_
#define SRC_TOOLS_H_
#include <vector>
#include "Eigen/Dense"

Eigen::VectorXd EvaluateRmse(const std::vector<Eigen::VectorXd> &estimations,
                             const std::vector<Eigen::VectorXd> &ground_truth);

Eigen::Vector3d PredictRadarMeasurement(const Eigen::Vector4d& x_state);
Eigen::Matrix3d PredictRadarMeasurementJac(const Eigen::Vector4d& x_state);

#endif  // SRC_TOOLS_H_
