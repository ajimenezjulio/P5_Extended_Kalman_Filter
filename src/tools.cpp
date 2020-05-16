#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Calculate Root Mean Square Error
 *
 * @param estimation, ground_truth values to be compared
 * @return rmse.
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Vector object to return
  VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;
  
  // Validate the input
  //  - The estimation vector size should not be zero
  //  - The estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0){
      std::cout << "Invalid estimation or ground_truth data" << std::endl;
      return rmse;
  }

  // Accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i){
      VectorXd residual = estimations[i] - ground_truth[i];

      // Coefficient-wise multiplication
      residual = residual.array() * residual.array();
      rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  // Return the result
  return rmse;
}

/**
 * Calculate Jacobian Matrix of the current state
 *
 * @param vector containing the current state
 * @return Jacobian Matrix.
 */
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Matrix object to return
  MatrixXd Hj(3,4);
  
  // Recover state parameters
  const double & px = x_state(0);
  const double & py = x_state(1);
  const double & vx = x_state(2);
  const double & vy = x_state(3);

  // Pre-compute a set of terms to avoid repeated calculation
  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  // Check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  // Compute the Jacobian matrix
  Hj <<  (px/c2), (py/c2), 0.0, 0.0,
        -(py/c1), (px/c1), 0.0, 0.0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
