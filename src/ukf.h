#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;
  ///* predicted sigma measurement matrix
  MatrixXd Zsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;
  long long previous_time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* NIS for Lidar
  std::vector<double> lidar_nis_;

  ///* NIS for Radar
  std::vector<double> radar_nis_;

  /**
   * Constructor
   */
  UKF(float a=30.0, float b=30.0);

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);
  /**
   * Make the augmented state from the current state.
   * @param Pointer to the returned augmented state vector.
   */
  void AugmentedSigmaPoints(MatrixXd* Xsig_out);
  /**
   * Make the prediction of the sigma points based on the plant model.
   * @param Pointer to the sigma points before
   * @param The delta time.
   * @param Pointer to the sigma points after
   */
  void SigmaPointPrediction(MatrixXd* Xsig_in, double delta_t, MatrixXd* Xsig_out);
  /**
   * Make the prediction of the mean and covariance based on the plant model.
   * @param Pointer to the sigma points before
   * @param The pointer to the mean state vector x
   * @param Pointer to the covariance matrix P
   */
  void PredictMeanAndCovariance(MatrixXd* Xsig_in, VectorXd* x_out, MatrixXd* P_out);
  /**
   * Make the prediction of the mean and covariance of the radar measurement.
   * @param Pointer to the sigma points before
   * @param The pointer to the mean measurement vector z
   * @param Pointer to the measurement covariance matrix R
   */
  void PredictRadarMeasurement(MatrixXd* Xsig_in, VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out);
  /**
   * Make the prediction of the mean and covariance of the lidar measurement.
   * @param Pointer to the sigma points before
   * @param The pointer to the mean measurement vector z
   * @param Pointer to the measurement covariance matrix R
   */
  void PredictLidarMeasurement(MatrixXd* Xsig_in, VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out);
  /**
   * Make the prediction of the mean and covariance of the radar measurement.
   * @param Pointer to the sigma points before
   * @param The pointer to the mean measurement vector z
   * @param Pointer to the measurement covariance matrix R
   */
  void UpdateStateRadar(MatrixXd* Xsig_in, VectorXd* z_in, VectorXd* z_pred_in, MatrixXd* Zsig_in,
			MatrixXd* S_in, VectorXd* x_out, MatrixXd* P_out);
  /**
   * Make the prediction of the mean and covariance of the lidar measurement.
   * @param Pointer to the sigma points before
   * @param The pointer to the mean measurement vector z
   * @param Pointer to the measurement covariance matrix R
   */
  void UpdateStateLidar(MatrixXd* Xsig_in, VectorXd* z_in, VectorXd* z_pred_in, MatrixXd* Zsig_in,
			MatrixXd* S_in, VectorXd* x_out, MatrixXd* P_out);
  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */

