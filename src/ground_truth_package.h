#ifndef SRC_GROUND_TRUTH_PACKAGE_H_
#define SRC_GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"
#include <cstdint>

class GroundTruthPackage {
 public:
  std::int64_t timestamp_;
  Eigen::VectorXd gt_values_;
};

#endif  // SRC_GROUND_TRUTH_PACKAGE_H_
