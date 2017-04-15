#ifndef SRC_KALMAN_FILTER_H_
#define SRC_KALMAN_FILTER_H_
#include "Eigen/Dense"

class IKalmanFilter {
 public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  virtual ~IKalmanFilter() = default;

  /**
   * Set Kalman Filter state and its noise covariance matrix
   * @param state_mean Initial state
   * @param state_cov Initial state covariance
   */
  virtual void SetState(const VectorXd& state_mean,
                        const MatrixXd& state_cov) = 0;

  /**
   * Perform prediction step
   * @param F Transition matrix
   * @param Q Process covariance matrix
   */
  virtual void Predict(const MatrixXd &F, const MatrixXd &Q) = 0;

  /**
   * Perform measurement step
   * @param y residual between current measurement and expected from the state
   * @param H Measurement matrix or measurement Jacobian
   * @param R Measurement covariance matrix
   */
  virtual void Update(const VectorXd &y,
                      const MatrixXd &H,
                      const MatrixXd &R) = 0;

  virtual const VectorXd& GetStateMean() const = 0;
};

class KalmanFilter : public IKalmanFilter {
 public:
  void SetState(const VectorXd& state_mean,
                const MatrixXd& state_cov) override;

  void Predict(const MatrixXd &F, const MatrixXd &Q) override;

  void Update(const VectorXd &y,
              const MatrixXd &H,
              const MatrixXd &R) override;

  const VectorXd& GetStateMean() const override;
  const MatrixXd& GetStateCov() const;

 private:
  VectorXd state_mean_;
  MatrixXd state_cov_;
};

#endif  // SRC_KALMAN_FILTER_H_
