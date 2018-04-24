
---

**Unscented Kalman Filter Project**

The goal of this project are the following:

Implement the unscented Kalman Filter for fusing the Lidar and Radar measurements to estimate the position of a tracked object from the car.

[//]: # (Image References)
[image1]: ./examples/tracked-points.png
[image2]: ./examples/nis.png


## [Rubric](https://review.udacity.com/#!/rubrics/783/view) Points
All the code for this project has been derived from the example code in the course and is in this directory.
[Here](https://github.com/gvp-study/CarND-Unscented-Kalman-Filter-Project.git)
---

### Unscented Kalman Filter

#### 1. Compiling
The code in the directory compiles without errors using cmake .. && make.

#### 2. Accuracy
The root mean squared error for the (px, py, vx, vy) is [0.071 0.083 0.343 0.225] <= [.09, .10, .40, .30].. This can be seen from the standard output when it runs ./UnscentedEKF program with the simulator running. The final values can also be seen in the right side of the figure below.

![alt text][image1]

#### 3. Follows the Correct Algorithm
I did not change the structure of the given code, so the sensor fusion algorithm flow is exactly as described in the lessons as shown in the figure below.

![alt text][image2]

I made sure that the state vector ekf_.x was initialized to the first given measurement. I also made sure that the covariance matrix ekf_.P was initialized to a diagonal matrix at the start with appropriate values for the position and velocity states.
The FusionEKF::ProcessMeasurements() function first predicts the state from the previous state using the plant matrix F and then updates the state with the new measurement based on the Kalman equations.
The update step uses the appropriate H and R matrices based on the kind of measurement it gets. If it is a Lidar measurement it uses the linear form of the kalman filter. If it is a Radar measurement, it uses the unscented kalman filter to update.

#### 4. Avoid Unnecessary Calculations

---

### Discussion

#### 5. Stand out
I have used the Q = G * A * Gt form to compute the Q matrix.

I put in the angle wrap check for the Radar phi angle measurement. When this measurement is subtracted from the estimated phi in the state vector, it could cause the value to be 2*pi if the phi is close to pi or -pi. This check keeps the difference in phi to be continuous near this value.
