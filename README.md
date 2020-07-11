# Object Tracking Using Extended Kalman Filter
This project is developed as part of Udacity Self Driving Car Engineer Nano Degree program.

<--- This is a work in progress --->

In this project a pedestrian is tracked using a Kalman filter. Only LASER measurements are used to track the pedestrian.

### Table of Contents
1. Kalman Filter
2. Project Description
3. How to Use the Project

## 1. Kalman filter
Kalman filter is basically a two-step estimation problem that consists of:

a. **State prediction**: use the available information to predict the state of the object (e.g., a robot, pedestrian, car) in 
consideration until the next measurement is available.

b. **Measurement update**: use new observations (a combination of the state prediction and new measurement) to correct the belief 
about the state of the object.

- Multiple sensors (e.g.,LASER, RADER) does the above state prediction and measurement update asynchronously.
- LASER provides measurements in Cartesian coordinates. If measurements are generated by LASER then we can apply a standard 
Kalman filter to update object state.
- RADAR provides measurement in POLAR coordinates. RADAR measurements involve a non-linear measurement function. 
So, different tricks are applied to handle measurement update, e.g., Extended Kalman filter.

## 2. Project Description
The main components of the project are:
- Eigen
- kalman_filter.cpp
- tracking.cpp
- measurement_package.h
- main.cpp

### Eigen
This is an open source package which is used for matrix and vector operations.

### kalman_filter.cpp
This file implements the KalmanFilter class with the following functions
- the Predict() function that predicts the location of a pedestrian.
- the Update() function that updates the predicted (by Predict() function) location of the pedestrian using 
the new LASER measurement.

The corresponding header file, kalman_filter.h declares the KalmanFilter class with its functions and parameters.

### tracking.cpp
This file implements the tracking class. 
- Using a constructor it initializes the state vector x_, covariance matrix P_,
measurement matrix H_, measurement covariance matrix R_, transition matrix F_ and acceleration nose components
noise_ax, noise_ay.
- ProcessMeasurement() function processes a single measurement. It initializes the state vector x_ if it is not initialized. 
Otherwise, it updates the transition matrix F_ with the elapsed time between the previous and current timestamps. It then
updates the process covariance matrix Q_ time and noise information.

### measurement_package.h
It creates the MeasurementPackage class with sensor type (RADAR and LASER) information. In this project only LASER 
measurement is used.

### main.cpp
In this file the Kalman filter is put into action.
- it reads the LASER measurement data from the file "obj_pose-laser-radar-synthetic-input.txt"
- for each measurement it tracks the pedestrian location

### How to use the project
Running the project is very straight forward:
- first, colne the project
$git clone https://github.com/moshiurmmr/Object_Tracking_Using_Extended_Kalman_Filter.git
- then run the main.cpp file. In a Linux platform.
$g++ main.cpp
- it should also run in any IDE. I have tried it in Visual Studio and it worked fine.
- *this project has been tested only in Ubuntu 18.04 LTS platform*
