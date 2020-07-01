#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter {
    public:
    // constructor
    KalmanFilter();

    // destructor
    virtual ~KalmanFilter();

    /*
    Predict() function calculates the state and the state covariance 
    using the process model
    */
   void Predict();

   /*
   Update() function updates the predicted state (calculated by the Predict()) with
   the new measurement
   */
  void Update(const VectorXd &z);

  // state vector
  VectorXd x_;

  // state covaiance matrix
  MatrixXd P_;

  // state transition matrix
  MatrixXd F_;

  // process covariance matrix
  MatrixXd Q_;

  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  MatrixXd R_;
};


#endif //KALMAN_FILTER_H_