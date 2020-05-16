#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

/**
 * Calculate Root Mean Square Error
 *
 * @param estimation, ground_truth values to be compared
 * @return rmse.
 */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

/**
 * Calculate Jacobian Matrix of the current state
 *
 * @param vector containing the current state
 * @return Jacobian Matrix.
 */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};

#endif  // TOOLS_H_
