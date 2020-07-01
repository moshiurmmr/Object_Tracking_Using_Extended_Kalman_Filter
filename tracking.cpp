#include "tracking.h"
#include <iostream>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

Tracking::Tracking(){
    is_initialzed_ = false;
    previous_timestamp_ = 0;

    // create a 4-D state vector with position (px, py) and velocity (vx, vy) components
    kf_.x_ = VectorXd(4);

    // state covariance matrix P_
    kf_.P_ = MatrixXd(4, 4);
    kf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

    // measurement matrix H_
    kf_.H_ = MatrixXd (2, 4);
    kf_.H_ << 1, 0, 0, 0,
              0, 1, 0, 0;
    
    // measurement covariance matrix R_
    kf_.R_ = MatrixXd(2, 2);
    kf_.R_ << 0.0225, 0,
              0, 0.0225;
    
    // initial transition matrix F_
    kf_.F_ = MatrixXd (4, 4);
    kf_.F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

    // set the acceleration noise components
    noise_ax = 5;
    noise_ay = 5;
}

// destructor 
Tracking::~Tracking() {

}

// process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack){
    // check if Kalman filter is initialized
    if(!is_initialzed_){
        cout << "Initialize Kalman filter" << endl;

        // set the set vector with initial position/location and zero velocity
        kf_.x_ << measurement_pack.raw_measurements_[0],
                  measurement_pack.raw_measurements_[1],
                  0,
                  0;
        previous_timestamp_ = measurement_pack.timestamp_;
        // after initialization set the is_initialized flag to true
        is_initialzed_ = true;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // Modify the F matrix so that the time is integrated
    kf_.F_ << 1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

    // set the process covariance matrix Q
    // square of dt
    float dt_2 = dt * dt;
    // cube of dt
    float dt_3 = dt_2 * dt;
    // 4-th order power of dt (dt^4)
    float dt_4 = dt_2 * dt_2;

    kf_.Q_ << dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
              0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
              dt_3/2 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;

}