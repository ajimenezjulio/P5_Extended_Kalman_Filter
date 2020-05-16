#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y      = z - z_pred;

  CommonUpdate(y);
}

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Parsing initial state
  const double & px = x_(0);
  const double & py = x_(1);
  const double & vx = x_(2);
  const double & vy = x_(3);
  
  // Convert from cartessian coordinates to polar ones
  double rho     = sqrt(px*px + py*py);
  double theta   = atan2(py, px);
  double rho_dot;
  
  // Check for rho close to 0
  if (fabs(rho) < 0.000001) {
    rho_dot = 0.000001;
  } else {
    rho_dot = (px * vx + py * vy) / rho;
  }

  VectorXd h = VectorXd(3);

  h << rho, theta, rho_dot;

  VectorXd y = z - h;

  // IMPORTANT - Normalize phi [-pi, pi] computing further matrices
  while (y(1) < -M_PI) y(1) += 2.0 * M_PI;
  while (y(1) > M_PI)  y(1) -= 2.0 * M_PI;
  
  CommonUpdate(y);
}

  /**
   * Common update procedure between Kalman and Extended Kalman filter
   * @param y Update function
   */
void KalmanFilter::CommonUpdate(const VectorXd &y){

  MatrixXd Ht  = H_.transpose();
  MatrixXd S   = H_ * P_ * Ht + R_;
  MatrixXd Si  = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K   = PHt * Si;

  // New estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
