#include <iostream>
#include <vector>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;


// Kalman filter variables
VectorXd x; // object state
MatrixXd P; // object covariance
VectorXd u; // external motion
MatrixXd F; // state transition matrix
MatrixXd H; // measurement matrix
MatrixXd R; // measurement covariance matrix
MatrixXd I; // identity matrix
MatrixXd Q; // process covariance matrix

vector <VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);

int main(){
    /* example codes for using in the Kalman filter calculation
    */
   // initialize variables for Kalman filter 
   x = VectorXd(2);
   x << 0, 0;
   //cout << "vector x is: " << x << endl;

   P = MatrixXd(2, 2);
   P << 1000, 0, 0, 1000;
   //cout << "Matrix P is: " << P << endl;

   u = VectorXd(2);
   u << 0, 0;

   F = MatrixXd(2, 2);
   F << 1, 1, 0, 1;

   H = MatrixXd(1, 2);
   H << 1, 0;

   R = MatrixXd(1,1);
   R << 1;

   I = MatrixXd::Identity(2, 2);

   Q = MatrixXd(2, 2);
   Q << 0, 0, 0, 0;

   // create a list of measurements
   VectorXd single_meas(1);
   single_meas << 1;
   measurements.push_back(single_meas);
   single_meas << 2;
   measurements.push_back(single_meas);
   single_meas << 3;
   measurements.push_back(single_meas);
   //single_meas << 5;
   //measurements.push_back(single_meas);

   // call Kalman filter
   filter(x, P);

   return 0;

}

// define the Kalman filter
void filter(VectorXd &x, MatrixXd &P){
    for (unsigned int n = 0; n < measurements.size(); ++n){
        VectorXd z = measurements[n];

        // Kalman filter measurement step
        VectorXd y = z - H * x;
        MatrixXd Ht = H.transpose();
        MatrixXd S = H * P * Ht + R;
        MatrixXd Si = S.inverse();
        MatrixXd K = P * Ht * Si;
        // new state after measurement
        x = x + ( K * y);
        P = (I - K * H) * P;

        // Kalman filter prediction state
        x = F * x + u;
        MatrixXd Ft = F.transpose();
        P = F * P * Ft + Q;

        cout << "x=" << endl << x << endl;
        cout << "P=" << endl << P << endl;
}

    }
    