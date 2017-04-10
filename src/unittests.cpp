#define CATCH_CONFIG_MAIN
#include "Catch/catch.hpp"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>

using std::vector;
using Eigen::Vector3d;

TEST_CASE("CalculateRMSE() test", "[tools][CalculateRMSE]") {
  const Vector3d rmse = CalculateRMSE({
    Vector3d{0.5, 0.2, 0.0},
    Vector3d{1.0, 0.3, -5.0}
  }, {
    Vector3d{-0.5, -1.8, 3.0},
    Vector3d{2.0, 2.3, -2.0}
  });

  REQUIRE(Approx(1.0) == rmse(0));
  REQUIRE(Approx(2.0) == rmse(1));
  REQUIRE(Approx(3.0) == rmse(2));
}
