#ifndef TRACKING_H_
#define TRACKING_H_

#include <vector>
#include <string>
#include <fstream>

#include "kalman_filter.h"
#include "measurement_package.h"

class Tracking
{
    public:
        // tracking constructor
        Tracking();
        // tracking destructor
        virtual ~Tracking();
        // ProcessMeasurement fuction
        void ProcessMeasurement(const MeasurementPackage &measurement_pack);
        KalmanFilter kf_;
    private:
        bool is_initialzed_;
        int64_t previous_timestamp_;

        // acceleration noise components
        float noise_ax;
        float noise_ay;


};

#endif