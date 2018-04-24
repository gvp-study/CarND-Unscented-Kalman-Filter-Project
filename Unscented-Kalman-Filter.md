
---

**Unscented Kalman Filter Project**

The goal of this project are the following:

Implement the unscented Kalman Filter for fusing the Lidar and Radar measurements to estimate the position of a tracked object from the car.

[//]: # (Image References)
[image1]: ./examples/tracked-points.png
[image2]: ./examples/radar-nis.png
[image3]: ./examples/laser-nis.png


## [Rubric](https://review.udacity.com/#!/rubrics/783/view) Points
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-Unscented-Kalman-Filter-Project.git)
---

### Unscented Kalman Filter

#### 1. Compiling
The code in the directory compiles without errors using cmake .. && make.

#### 2. Accuracy
After some experimentation with the values of the standard deviation of the error in the linear and angular accelerations, I found a good value to be std_a=2.0 and std_yawdd=0.5.
The root mean squared error for the DataSet1 (px, py, vx, vy) with these error values is [0.07 0.08 0.34 0.23] <= [.09, .10, .40, .30]. This can be seen from the standard output when it runs ./UnscentedEKF program with the simulator running. The final values can also be seen in the right side of the figure below.
I also tried the run with the second DataSet2 and the resulting MSEE was [0.09 0.07 0.66 0.31].

![alt text][image1]

I plotted the Normalized Innovation Squared value and observed that 95% of them stayed within the sigma limits.

The NIS values for the innovations from the radar with 3 degrees of freedom are under the 7.8 for 95% of the values as shown below.

![alt text][image2]

The NIS values for the innovations from the laser with 2 degrees of freedom are under the 5.99 for 95% of the values as shown below.

![alt text][image3]
#### 3. Follows the Correct Algorithm
I did not change the structure of the given code, so the sensor fusion algorithm flow is exactly as described in the lessons.

I made sure that the state vector x_ was initialized to the first given measurement. I also made sure that the covariance matrix P_ was initialized to an identity matrix at the start with appropriate values for the position and velocity states.

The UKF::ProcessMeasurements() function first predicts the state from the previous state using the plant equations and then updates the state with the new measurement based on the Kalman equations.
The update step uses UpdateLidar when the measurement is from the laser and UpdateRadar when the measurement is from the radar.

Unlike the extended kalman filter, the UKF uses the sigma points to predict the apriori state and covariance in the prediction step. It then uses the predicted sigma points to predict the measurements and updates the state and covariance based on the innovation seen from the actual measurement in the update step.

#### 4. Avoid Unnecessary Calculations
I kept the weights as a member variable for the UKF class so that it was computed once at start and used multiple times later during prediction and update cycles.

---

### Discussion

#### 5. Stand out

I made some changes to allow for the setting of the std_a and std_yawdd to be set from the command line. This allowed me to experiment with different values of these system noise parameters without changing the code.
I also made the program write out the MSEE error and NIS values to appropriate files before exiting the program. This allowed me to plot the results after using python.

#### 6. Problems
I was unable to run the runaway car catching part of the simulator because it was crashing my program.
