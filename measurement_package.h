#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H

#include "Eigen/Dense"

class MeasurementPackage {
    public:
        enum SensorType{
            LASER, LIDAR
        } sensor_type_;

        Eigen::VectorXd raw_measurements_;
        int64_t timestamp_;
};


#endif