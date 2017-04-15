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
  void SetState(const VectorXd& state_mean,
                const MatrixXd& state_cov) override {
    state_mean_ = state_mean;
    state_cov_ = state_cov;
    ++set_state_calls_;
  }

  void Predict(const MatrixXd &F, const MatrixXd &Q) override {
    F_ = F;
    Q_ = Q;
    ++predict_calls_;
  }

  void Update(const VectorXd &y,
              const MatrixXd &H,
              const MatrixXd &R) override {
    y_ = y;
    H_ = H;
    R_ = R;
    ++update_calls_;
  }

  const VectorXd& GetStateMean() const override {
    return state_mean_;
  }

  VectorXd state_mean_{};
  MatrixXd state_cov_{};
  int set_state_calls_{0};

  MatrixXd F_{};
  MatrixXd Q_{};
  int predict_calls_{0};

  VectorXd y_{};
  MatrixXd H_{};
  MatrixXd R_{};
  int update_calls_{0};
};

TEST_CASE("FusionEKF tests", "[tools][FusionEKF]") {
  KalmanFilterStub stub;
  FusionEKF fusion(&stub);

  SECTION("Lidar Measurement") {
    MeasurementPackage measurement;
    measurement.sensor_type_ = MeasurementPackage::LASER;
    measurement.timestamp_ = 0;
    measurement.raw_measurements_ = Eigen::Vector2d{10, 20};
    fusion.ProcessMeasurement(measurement);

    REQUIRE(0 == stub.update_calls_);
    REQUIRE(0 == stub.predict_calls_);
    REQUIRE(1 == stub.set_state_calls_);
    REQUIRE(4 == stub.state_mean_.size());
    REQUIRE(4 == stub.state_cov_.rows());
    REQUIRE(4 == stub.state_cov_.cols());
    REQUIRE(Approx(10) == stub.state_mean_(0));
    REQUIRE(Approx(20) == stub.state_mean_(1));

    SECTION("Second Lidar Measurement") {
      MeasurementPackage measurement;
      measurement.sensor_type_ = MeasurementPackage::LASER;
      measurement.timestamp_ = 10000000;
      measurement.raw_measurements_ = Eigen::Vector2d{20, 20};
      fusion.ProcessMeasurement(measurement);

      REQUIRE(1 == stub.predict_calls_);

      Eigen::Matrix4d expected_F;
      expected_F <<
        1.0, 0.0, 10.0, 0.0,
        0.0, 1.0, 0.0, 10.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

      REQUIRE(stub.F_.isApprox(expected_F));
      REQUIRE(stub.Q_.isApprox(stub.Q_.transpose()));
      REQUIRE(stub.Q_.eigenvalues()[0].real() > 1e-10);

      REQUIRE(1 == stub.update_calls_);
      REQUIRE(Approx(10) == stub.y_(0));
      REQUIRE(Approx(0) == stub.y_(1));
      REQUIRE(2 == stub.H_.rows());
      REQUIRE(4 == stub.H_.cols());
      REQUIRE(2 == stub.R_.rows());
      REQUIRE(2 == stub.R_.cols());
    }

    SECTION("Second Radar Measurement") {
      MeasurementPackage measurement;
      measurement.sensor_type_ = MeasurementPackage::RADAR;
      measurement.timestamp_ = 1000000;
      measurement.raw_measurements_ = Eigen::Vector3d{20, M_PI / 2, 1.0};
      fusion.ProcessMeasurement(measurement);

      REQUIRE(1 == stub.update_calls_);
      REQUIRE(3 == stub.H_.rows());
      REQUIRE(4 == stub.H_.cols());
      REQUIRE(3 == stub.R_.rows());
      REQUIRE(3 == stub.R_.cols());

      REQUIRE(stub.state_cov_.isApprox(stub.state_cov_.transpose()));
      REQUIRE(stub.state_cov_.eigenvalues()[0].real() > 1e-10);
    }
  }

  SECTION("Radar Measurement") {
    MeasurementPackage measurement;
    measurement.sensor_type_ = MeasurementPackage::RADAR;
    measurement.timestamp_ = 0;
    measurement.raw_measurements_ = Eigen::Vector3d{1.0, 0.0, 3.0};
    fusion.ProcessMeasurement(measurement);

    REQUIRE(0 == stub.update_calls_);
    REQUIRE(1 == stub.set_state_calls_);
    REQUIRE(4 == stub.state_mean_.size());
    REQUIRE(4 == stub.state_cov_.rows());
    REQUIRE(4 == stub.state_cov_.cols());
    REQUIRE(Approx(1) == stub.state_mean_(0));
    REQUIRE(Approx(0) == stub.state_mean_(1));
    REQUIRE(Approx(3) == stub.state_mean_(2));
    REQUIRE(Approx(0) == stub.state_mean_(3));
  }

  SECTION("0-ro Radar Measurement") {
    MeasurementPackage measurement;
    measurement.sensor_type_ = MeasurementPackage::RADAR;
    measurement.timestamp_ = 0;
    measurement.raw_measurements_ = Eigen::Vector3d{0.0, 1.0, 3.0};
    fusion.ProcessMeasurement(measurement);

    REQUIRE(0 == stub.update_calls_);
    REQUIRE(1 == stub.set_state_calls_);
    REQUIRE(4 == stub.state_mean_.size());
    REQUIRE(4 == stub.state_cov_.rows());
    REQUIRE(4 == stub.state_cov_.cols());
    REQUIRE(Approx(0) == stub.state_mean_(0));
    REQUIRE(Approx(0) == stub.state_mean_(1));
    REQUIRE(Approx(0) == stub.state_mean_(2));
    REQUIRE(Approx(0) == stub.state_mean_(3));

    SECTION("Radar Measurement for (0, 0) state is not skipped") {
      measurement.timestamp_ = 1000000;
      measurement.raw_measurements_ = Eigen::Vector3d{1.0, 2.0, 3.0};
      fusion.ProcessMeasurement(measurement);
      REQUIRE(0 == stub.update_calls_);
    }
  }

  SECTION("Correct RADAR Angle Warping") {
    const double angle_offset = 0.001;
    const double angle0 = M_PI - angle_offset / 2;
    const double angle1 = -M_PI + angle_offset / 2;

    MeasurementPackage measurement;
    measurement.sensor_type_ = MeasurementPackage::RADAR;

    SECTION("Counter clockwise") {
      measurement.timestamp_ = 0;
      measurement.raw_measurements_ = Eigen::Vector3d{1.0, angle0, 0.0};
      fusion.ProcessMeasurement(measurement);

      measurement.timestamp_ = 1000000;
      measurement.raw_measurements_ = Eigen::Vector3d{1.0, angle1, 0.0};
      fusion.ProcessMeasurement(measurement);

      REQUIRE(1 == stub.update_calls_);
      REQUIRE(Approx(angle_offset) == stub.y_[1]);
    }

    SECTION("Clockwise") {
      measurement.timestamp_ = 0;
      measurement.raw_measurements_ = Eigen::Vector3d{1.0, angle1, 0.0};
      fusion.ProcessMeasurement(measurement);

      measurement.timestamp_ = 1000000;
      measurement.raw_measurements_ = Eigen::Vector3d{1.0, angle0, 0.0};
      fusion.ProcessMeasurement(measurement);

      REQUIRE(1 == stub.update_calls_);
      REQUIRE(Approx(-angle_offset) == stub.y_[1]);
    }
  }
}

TEST_CASE("Kalman Filter tests", "[tools][KalmanFilter][kalman_filter]") {
  SECTION("1D position and speed") {
    using Eigen::Vector2d;
    using Eigen::Matrix2d;

    KalmanFilter kf;

    const Matrix2d initial_state_cov = Matrix2d::Identity() * 100;

    kf.SetState(Vector2d{-1.0, 2.0}, initial_state_cov);

    SECTION("Test Prediction") {
      Matrix2d F;
      F <<
        1.0, 1.0,
        0.0, 1.0;
      kf.Predict(F, Matrix2d::Identity() * 10);

      {
        const auto& state_mean = kf.GetStateMean();
        REQUIRE(Approx(1.0) == state_mean(0));
        REQUIRE(Approx(2.0) == state_mean(1));

        const auto& state_cov = kf.GetStateCov();
        REQUIRE(1.0 < state_cov(0, 1));
        REQUIRE(Approx(state_cov(1, 0)) == state_cov(0, 1));
        REQUIRE(initial_state_cov(0, 0) < state_cov(0, 0));
        REQUIRE(initial_state_cov(1, 1) < state_cov(1, 1));
      }

      SECTION("Test Measurement Position Update") {
        Eigen::MatrixXd H(1, 2);
        H << 1.0, 0.0;

        Eigen::VectorXd z(1);
        z << 10.0;

        Eigen::MatrixXd R(1, 1);
        R << 0.0001;

        const Eigen::VectorXd y = z - H * kf.GetStateMean();
        kf.Update(y, H, R);

        {
          const auto& state_mean = kf.GetStateMean();
          REQUIRE(Approx(10.0) == state_mean(0));

          const auto& state_cov = kf.GetStateCov();
          REQUIRE(Approx(state_cov(1, 0)) == state_cov(0, 1));
          REQUIRE(1e-5 < state_cov(0, 0));
          REQUIRE(1e-5 < state_cov(1, 1));
          REQUIRE(initial_state_cov(0, 0) > state_cov(0, 0));
          REQUIRE(initial_state_cov(1, 1) > state_cov(1, 1));
          REQUIRE(R(0, 0) > state_cov(0, 0));
        }
      }
    }
  }
}
