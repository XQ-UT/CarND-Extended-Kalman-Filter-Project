# Extended Kalman Filter Project

This project implements extened Kalman filter algorithm based on lidar
and radar sensor data.

## Algorithm

The EKF algorithm consists of prediction and measurement update steps.
The intuition is predicting object's state based on previous estimation
and then correcting our prediction based on sensor measurement.

For lidar measurement, Hx function is linear. So we can apply the Kalman
filter algorithm directly. For radar measurement, hx is non linear. Thus,
we need to map our state to polar coordinates and derive Tyler Expansion for Hx.

<p align="center">
  <img src="report_images/algorithm.png" width="500" height="300"/>
  <br>
  <em>Figure 1: EKF Algorithm</em>
</p>

## Result

The final tracking result and RMSE is showed in Figure 2.

<p align="center">
  <img src="report_images/result.png" width="500" height="300"/>
  <br>
  <em>Figure 2: Final Result</em>
</p>
