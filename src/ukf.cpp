#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  cout << "UKF Initialing" << endl;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  
  // Plant Covariance
  P_ = MatrixXd::Identity(n_x_, n_x_);
  // Sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_x_ + 1);
  Xsig_pred_.setZero();

  cout << "UKF Initialized" << endl;
  cout << "x" << x_ << endl;
  cout << "P" << endl << P_ << endl;
  cout << "Xsig_pred" << endl << Xsig_pred_ << endl;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  time_us_ = measurement_pack.timestamp_;
  // Find the delta t
  double dt = (time_us_ - previous_time_us_) / 1000000.0;	//dt - expressed in seconds
  if(!is_initialized_)
  {
    // Try to initialize from the first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //
      //Convert radar from polar to cartesian coordinates and initialize state.
      //
      double r = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rdot = measurement_pack.raw_measurements_[2];
      x_(0) = r * cos(phi);
      x_(1) = r * sin(phi);
      x_(2) = rdot;
      x_(3) = phi;
      x_(4) = 0.0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //
      //Initialize state.
      //
      x_(0) = measurement_pack.raw_measurements_[0];
      x_(1) = measurement_pack.raw_measurements_[1];
      x_(2) = 0.0;
      x_(3) = 0.0;
      x_(4) = 0.0;
    }

    dt = 0.0;
    is_initialized_ = true;
  }
  previous_time_us_ = measurement_pack.timestamp_;
  cout << "Now " << time_us_ << " Old " << previous_time_us_ << " DT " << dt << endl;
  //
  // Predict using the delta t
  //
  Prediction(dt);
  //
  // Update using measurements.
  //
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    UpdateRadar(measurement_pack);
  } else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    // Laser updates
    UpdateLidar(measurement_pack);
  }
  
  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // Make the augmented sigma points matrix from the current state.
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  MatrixXd Xsig_pred;
  AugmentedSigmaPoints(&Xsig_aug);
  cout << "XAug " << endl << Xsig_aug << endl;
  SigmaPointPrediction(&Xsig_aug, delta_t, &Xsig_pred);
  cout << "X Predicted for time " << delta_t << endl <<  Xsig_pred << endl;
//  x_ = Xsig_pred.col(0);
  VectorXd x_out;
  MatrixXd P_out;
  PredictMeanAndCovariance(&Xsig_pred, &x_out, &P_out);
  x_ = x_out;
  P_ = P_out;
  Xsig_pred_ = Xsig_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  cout << "LIDAR" << endl;
  /**
     Convert radar from polar to cartesian coordinates and initialize state.
  */
  int n_z = 2;
  VectorXd z_pred(n_z);
  MatrixXd S_pred(n_z, n_z);
  MatrixXd Xsig_in = Xsig_pred_;
  MatrixXd Zsig_pred;
  PredictLidarMeasurement(&Xsig_in, &z_pred, &S_pred, &Zsig_pred);
  double lx = measurement_pack.raw_measurements_[0];
  double ly = measurement_pack.raw_measurements_[1];
  VectorXd z_in(2);
  z_in << lx, ly;
  VectorXd x_out;
  MatrixXd P_out;
  UpdateStateLidar(&Xsig_in, &z_in, &z_pred, &Zsig_pred, &S_pred, &x_out, &P_out);

  x_ = x_out;
  P_ = P_out;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} measurement_pack
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  cout << "RADAR" << endl;
  /**
     Convert radar from polar to cartesian coordinates and initialize state.
  */
  int n_z = 3;
  VectorXd z_pred(n_z);
  MatrixXd S_pred(n_z, n_z);
  MatrixXd Xsig_in = Xsig_pred_;
  MatrixXd Zsig_pred;
  PredictRadarMeasurement(&Xsig_in, &z_pred, &S_pred, &Zsig_pred);
  double r = measurement_pack.raw_measurements_[0];
  double phi = measurement_pack.raw_measurements_[1];
  double rdot = measurement_pack.raw_measurements_[2];
  VectorXd z_in(3);
  z_in << r, phi, rdot;
  VectorXd x_out;
  MatrixXd P_out;
  UpdateStateRadar(&Xsig_in, &z_in, &z_pred, &Zsig_pred, &S_pred, &x_out, &P_out);

  x_ = x_out;
  P_ = P_out;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = n_x_;

  //set augmented dimension
  int n_aug = n_aug_;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = std_a_;

  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = std_yawdd_;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set example state
  VectorXd x = x_;

  //create example covariance matrix
  MatrixXd P = P_;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);
  P_aug.setZero();

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug.setZero();

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
 
  //create augmented mean state
  x_aug.head(n_x) = x;
  x_aug(5) = 0.0;
  x_aug(6) = 0.0;
  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x, n_x) = P;
  MatrixXd Q(2, 2); 
  Q << std_a * std_a, 0.0,
  0.0, std_yawdd * std_yawdd;
  P_aug.bottomRightCorner(2, 2) = Q;
  //create square root matrix
  MatrixXd S = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug; i++)
  {
    double factor = sqrt(n_aug+lambda);
    Xsig_aug.col(i+1)       = x_aug + factor * S.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - factor * S.col(i);
  }
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
  std::cout << "P_aug = " << std::endl << P_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;

}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_in, double delta_t, MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = n_x_;

  //set augmented dimension
  int n_aug = n_aug_;

  //create example sigma point matrix
  MatrixXd Xsig_aug = *(Xsig_in);

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //predict sigma points
  int n = 2*n_aug + 1;
  for(int i = 0; i < n; i++)
  {
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);
    // predicted state 
    double px_p, py_p;
    
    //avoid division by zero
    if(fabs(yawd) > 0.001)
    {
        px_p = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = py + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    }
    else
    {
        px_p = px + v * delta_t * cos(yaw);
        py_p = py + v * delta_t * sin(yaw);
    }
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
    
    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
    
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
    //write predicted sigma points into right column
    Xsig_pred(0, i) = px_p;
    Xsig_pred(1, i) = py_p;
    Xsig_pred(2, i) = v_p;
    Xsig_pred(3, i) = yaw_p;
    Xsig_pred(4, i) = yawd_p;
  }
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;

}

void UKF::PredictMeanAndCovariance(MatrixXd* Xsig_in, VectorXd* x_out, MatrixXd* P_out) {

  //set state dimension
  int n_x = n_x_;

  //set augmented dimension
  int n_aug = n_aug_;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = *Xsig_in;

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  
  //create vector for predicted state
  VectorXd x = VectorXd(n_x);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x, n_x);


/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  // set weights
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    x = x + weights(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }


/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
}

void UKF::PredictRadarMeasurement(MatrixXd* Xsig_in, VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //set augmented dimension
  int n_aug = n_aug_;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }
  weights_ = weights;
  //radar measurement noise standard deviation radius in m
  double std_radr = std_radr_;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = std_radphi_;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = std_radrd_;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = *Xsig_in;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  Zsig.setZero();

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    if(hypot(p_x, p_y) > 0.001)
    {
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
      Zsig(1,i) = atan2(p_y,p_x);                                 //phi
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }
  }
  std::cout << "Zsig Radar: " << std::endl << Zsig << std::endl;
  
  *Zsig_out = Zsig;
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;
  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "z_pred Radar: " << std::endl << z_pred << std::endl;
  std::cout << "S Radar: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
}


void UKF::UpdateStateRadar(MatrixXd* Xsig_in, VectorXd* z_in, VectorXd* z_pred_in, MatrixXd* Zsig_in,
			   MatrixXd* S_in, VectorXd* x_out, MatrixXd* P_out) {

  //set state dimension
  int n_x = n_x_;

  //set augmented dimension
  int n_aug = n_aug_;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = *Xsig_in;
  //create example vector for predicted state mean
  VectorXd x = x_;

  //create example matrix for predicted state covariance
  MatrixXd P = P_;

  //create example matrix with sigma points in measurement space
  MatrixXd Zsig = *Zsig_in;

  //create example vector for mean predicted measurement
  VectorXd z_pred = *z_pred_in;

  //create example matrix for predicted measurement covariance
  MatrixXd S = *S_in;

  //create example vector for incoming radar measurement
  VectorXd z = *z_in;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;

}
void UKF::PredictLidarMeasurement(MatrixXd* Xsig_in, VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //set augmented dimension
  int n_aug = n_aug_;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }
  weights_ = weights;
  //laser measurement noise standard deviation in m in X.
  double std_laspx = std_laspx_;

  //laser measurement noise standard deviation in m in Y.
  double std_laspy = std_laspy_;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = *Xsig_in;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  Zsig.setZero();

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }
  std::cout << "Zsig Laser: " << std::endl << Zsig << std::endl;
  
  *Zsig_out = Zsig;
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_laspx*std_laspx, 0,
          0, std_laspy*std_laspy;

  S = S + R;
  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "z_pred Laser: " << std::endl << z_pred << std::endl;
  std::cout << "S Laser: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
}

void UKF::UpdateStateLidar(MatrixXd* Xsig_in, VectorXd* z_in, VectorXd* z_pred_in, MatrixXd* Zsig_in,
			   MatrixXd* S_in, VectorXd* x_out, MatrixXd* P_out) {

  //set state dimension
  int n_x = n_x_;

  //set augmented dimension
  int n_aug = n_aug_;

  //set measurement dimension, lidar return x and y
  int n_z = 2;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = *Xsig_in;
  //create example vector for predicted state mean
  VectorXd x = x_;

  //create example matrix for predicted state covariance
  MatrixXd P = P_;

  //create example matrix with sigma points in measurement space
  MatrixXd Zsig = *Zsig_in;

  //create example vector for mean predicted measurement
  VectorXd z_pred = *z_pred_in;

  //create example matrix for predicted measurement covariance
  MatrixXd S = *S_in;

  //create example vector for incoming radar measurement
  VectorXd z = *z_in;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;

}

