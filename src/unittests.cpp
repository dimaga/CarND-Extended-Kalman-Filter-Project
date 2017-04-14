#define CATCH_CONFIG_MAIN
#include "Catch/catch.hpp"
#include "tools.h"
#include "FusionEKF.h"
#include "kalman_filter.h"
#include "Eigen/Dense"
#include <vector>
#include <cmath>

using std::vector;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;

TEST_CASE("EvaluateRmse() tests", "[tools][EvaluateRmse]") {
  const Vector3d rmse = EvaluateRmse({
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

TEST_CASE("PredictRadarMeasurement() tests",
          "[tools][PredictRadarMeasurement]") {
  Vector3d measurement{Vector3d::Zero()};

  SECTION("Typical value") {
    REQUIRE(PredictRadarMeasurement({-1.0, 1.0, 4.0, 2.0}, &measurement));
    REQUIRE(Approx(std::sqrt(2.0)) == measurement[0]);
    REQUIRE(Approx(3 * M_PI / 4) == measurement[1]);
    REQUIRE(Approx(-std::sqrt(2.0)) == measurement[2]);
  }

  SECTION("Boundary value") {
    REQUIRE(!PredictRadarMeasurement({0.0, 0.0, 1.0, 2.0}, &measurement));
  }
}

TEST_CASE("PredictRadarMeasurementJac() tests",
          "[tools][PredictRadarMeasurementJac]") {
  MatrixXd jac;

  SECTION("Typical value") {
    const Vector4d v{1.2, 3.4, -2.0, 1.0};
    REQUIRE(PredictRadarMeasurementJac(v, &jac));
    REQUIRE(3 == jac.rows());
    REQUIRE(4 == jac.cols());

    const double kEps = 1e-10;
    Vector3d m1{Vector3d::Zero()};
    Vector3d m0{Vector3d::Zero()};

    SECTION("px grad") {
      REQUIRE(PredictRadarMeasurement(v + Vector4d{kEps, 0.0, 0.0, 0.0}, &m1));
      REQUIRE(PredictRadarMeasurement(v - Vector4d{kEps, 0.0, 0.0, 0.0}, &m0));
      const auto grad = (m1 - m0) / (2.0 * kEps);
      REQUIRE(Approx(grad(0)) == jac(0, 0));
      REQUIRE(Approx(grad(1)) == jac(1, 0));
      REQUIRE(Approx(grad(2)) == jac(2, 0));
    }

    SECTION("py grad") {
      REQUIRE(PredictRadarMeasurement(v + Vector4d{0.0, kEps, 0.0, 0.0}, &m1));
      REQUIRE(PredictRadarMeasurement(v - Vector4d{0.0, kEps, 0.0, 0.0}, &m0));
      const auto grad = (m1 - m0) / (2.0 * kEps);
      REQUIRE(Approx(grad(0)) == jac(0, 1));
      REQUIRE(Approx(grad(1)) == jac(1, 1));
      REQUIRE(Approx(grad(2)) == jac(2, 1));
    }

    SECTION("vx grad") {
      REQUIRE(PredictRadarMeasurement(v + Vector4d{0.0, 0.0, kEps, 0.0}, &m1));
      REQUIRE(PredictRadarMeasurement(v - Vector4d{0.0, 0.0, kEps, 0.0}, &m0));
      const auto grad = (m1 - m0) / (2.0 * kEps);
      REQUIRE(Approx(grad(0)) == jac(0, 2));
      REQUIRE(Approx(grad(1)) == jac(1, 2));
      REQUIRE(Approx(grad(2)) == jac(2, 2));
    }

    SECTION("vy grad") {
      REQUIRE(PredictRadarMeasurement(v + Vector4d{0.0, 0.0, 0.0, kEps}, &m1));
      REQUIRE(PredictRadarMeasurement(v - Vector4d{0.0, 0.0, 0.0, kEps}, &m0));
      const auto grad = (m1 - m0) / (2.0 * kEps);
      REQUIRE(Approx(grad(0)) == jac(0, 3));
      REQUIRE(Approx(grad(1)) == jac(1, 3));
      REQUIRE(Approx(grad(2)) == jac(2, 3));
    }
  }

  SECTION("Boundary value") {
    REQUIRE(!PredictRadarMeasurementJac({0.0, 0.0, 1.0, 2.0}, &jac));
  }
}

class KalmanFilterStub : public IKalmanFilter {
 public:
  void Init(const Eigen::VectorXd &x_in,
            const Eigen::MatrixXd &P_in,
            const Eigen::MatrixXd &F_in,
            const Eigen::MatrixXd &H_in,
            const Eigen::MatrixXd &R_in,
            const Eigen::MatrixXd &Q_in) override {
  }

  void Predict() override {
  }

  void Update(const Eigen::VectorXd &z) override {
  }

  void UpdateEKF(const Eigen::VectorXd &z) override {
  }

  void SetState(const Eigen::VectorXd& state_mean,
                const Eigen::MatrixXd& state_cov) override {
  }
};

TEST_CASE("FusionEKF tests", "[tools][FusionEKF]") {
  KalmanFilterStub stub;
  FusionEKF fusion(&stub);
}
