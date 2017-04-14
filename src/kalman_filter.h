#ifndef SRC_KALMAN_FILTER_H_
#define SRC_KALMAN_FILTER_H_
#include "Eigen/Dense"

class IKalmanFilter {
 public:
  virtual ~IKalmanFilter() = default;

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  virtual void Init(const Eigen::VectorXd &x_in,
                    const Eigen::MatrixXd &P_in,
                    const Eigen::MatrixXd &F_in,
                    const Eigen::MatrixXd &H_in,
                    const Eigen::MatrixXd &R_in,
                    const Eigen::MatrixXd &Q_in) = 0;

  virtual void Predict() = 0;
  virtual void Update(const Eigen::VectorXd &z) = 0;
  virtual void UpdateEKF(const Eigen::VectorXd &z) = 0;

  virtual void SetState(const Eigen::VectorXd& state_mean,
                        const Eigen::MatrixXd& state_cov) = 0;
};

class KalmanFilter : public IKalmanFilter {
 public:
  void Init(const Eigen::VectorXd &x_in,
            const Eigen::MatrixXd &P_in,
            const Eigen::MatrixXd &F_in,
            const Eigen::MatrixXd &H_in,
            const Eigen::MatrixXd &R_in,
            const Eigen::MatrixXd &Q_in) override;

  void Predict() override;
  void Update(const Eigen::VectorXd &z) override;
  void UpdateEKF(const Eigen::VectorXd &z) override;

  void SetState(const Eigen::VectorXd& state_mean,
                const Eigen::MatrixXd& state_cov) override;

  Eigen::VectorXd GetStateMean() const;
  Eigen::MatrixXd GetStateCov() const;

 private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
};

#endif  // SRC_KALMAN_FILTER_H_
