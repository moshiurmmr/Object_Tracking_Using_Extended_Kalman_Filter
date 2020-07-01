#include "kalman_filter.h"

// constructor
KalmanFilter::KalmanFilter() {

}

// destructor
KalmanFilter::~KalmanFilter(){

}

// predict function
void KalmanFilter::Predict(){
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

// measurement update function
void KalmanFilter::Update(const VectorXd &z){
    // predicted state
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht  + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    // new estimate of state
    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}