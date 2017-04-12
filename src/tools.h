#ifndef SRC_TOOLS_H_
#define SRC_TOOLS_H_
#include <vector>
#include "Eigen/Dense"

Eigen::VectorXd EvaluateRmse(const std::vector<Eigen::VectorXd> &estimations,
                             const std::vector<Eigen::VectorXd> &ground_truth);

bool PredictRadarMeasurement(const Eigen::Vector4d& x_state,
                             Eigen::Vector3d* pMeasurement);

bool PredictRadarMeasurementJac(const Eigen::Vector4d& x_state,
                                Eigen::MatrixXd* pMeasurementJac);

#endif  // SRC_TOOLS_H_
