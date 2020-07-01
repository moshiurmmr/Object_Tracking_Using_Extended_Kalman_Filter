# Object Tracking Using Extended Kalman Filter
This project is developed as part of Udacity Self Driving Car Engineer Nano Degree program.

<--- This is a work in progress --->

In this project a pedestrian is tracked using a Kalman filter. Only LASER measurements are used to track the pedestrian.

## Kalman filter
Kalman filter is basically a two-step estimation problem that consists of:
**State prediction**: use the available information to predict the state of the object (e.g., a robot, pedestrian, car) in consideration until the next measurement is available
**Measurement update**: use new observations (a combination of the state prediction and new measurement) to correct the belief about the state of the object
- Multiple sensors (e.g.,LASER, RADER) does the above state prediction and measurement update asynchronously.
- LASER provides measurements in Cartesian coordinates. If measurements are generated by LASER then we can apply a standard Kalman filter to update object state.
- RADAR provides measurement in POLAR coordinates. RADAR measurements involve a non-linear measurement function. So, different tricks are applied to handle measurement update, e.g., Extended Kalman filter.
