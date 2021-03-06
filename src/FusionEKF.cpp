#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser
  R_laser_ << 0.0225, 0.0,
              0.0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ << 0.09, 0.0, 0.0,
              0.0, 0.0009, 0.0,
              0.0, 0.0, 0.09;
  
  // Measurement matrix - laser
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
  
  // Noise covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);
  
  // State transition matrix update. The ones outside the identity diagonal
  // will be replaced to Δt in further steps.
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1.0, 0.0, 1.0, 0.0,
             0.0, 1.0, 0.0, 1.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

  // State covariance Matrix
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1.0,  0.0,   0.0,    0.0,
             0.0,  1.0,   0.0,    0.0,
             0.0,  0.0,   1000.0, 0.0,
             0.0,  0.0,   0.0,    1000.0;

  // Measurement noise
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

/**
  * Run the whole flow of the Kalman Filter from here.
  * @param measurement_pack Measurement package given by sensor
  */
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // First measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1.0, 1.0, 1.0, 1.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
	  double rho     = measurement_pack.raw_measurements_(0);
      double phi     = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);
      
      // Normalize phi [-pi, pi]
      while (phi < -M_PI) phi += 2.0 * M_PI;
      while (phi > M_PI)  phi -= 2.0 * M_PI;
      
      // In this order x, y, vx, vy
      ekf_.x_(0) = rho     * cos(phi);
      ekf_.x_(1) = rho     * sin(phi);      
      ekf_.x_(2) = rho_dot * cos(phi);
      ekf_.x_(3) = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      // In this order x, y, vx, vy
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      ekf_.x_(2) = 0.0;
      ekf_.x_(3) = 0.0;
    }
	
    // Save initial timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // Compute the time elapsed between the current and previous measurements in seconds
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  ekf_.F_ << 1.0,  0.0,  dt,  0.0,
             0.0,  1.0,  0.0,  dt,
             0.0,  0.0,  1.0, 0.0,
             0.0,  0.0,  0.0, 1.0;
  
  // Pre-calculate some variables for the matrix
  double dt_2   = dt * dt;
  double dt_3   = dt_2 * dt;
  double dt_3_2 = dt_3 / 2;
  double dt_4   = dt_3 * dt;
  double dt_4_4 = dt_4 / 4;
  
  // Set noise covariance matrix
  ekf_.Q_ << dt_4_4*noise_ax_, 0.0,               dt_3_2*noise_ax_, 0.0,
             0.0,               dt_4_4*noise_ay_, 0.0,               dt_3_2*noise_ay_,
             dt_3_2*noise_ax_, 0.0,               dt_2*noise_ax_,   0.0,
             0.0,               dt_3_2*noise_ay_, 0.0,               dt_2*noise_ay_;
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
	ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
