#include "ukf.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 1, 1, 1, 1, 1;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 0.1, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.2; 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.35; 
  
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

  is_initialized_ = false;
  time_us_ = 0;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  // if it is the very first measurement
  if (is_initialized_ == false) {
    time_us_ = meas_package.timestamp_;
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
      */
      VectorXd measurements = meas_package.raw_measurements_;
      float rho = measurements[0];
      float theta = measurements[1];
      x_[0] = rho * cos(theta);
      x_[1] = rho * sin(theta);
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
       Initialize state.
      */
      VectorXd measurements = meas_package.raw_measurements_;
      x_[0] = measurements[0];
      x_[1] = measurements[1];
    }
    
    weights_ = VectorXd(2*n_aug_+1);
    weights_(0) = lambda_ / (lambda_+n_aug_);
    for (int i = 1; i < 2*n_aug_+1; i++){
      weights_(i) = 0.5  / (lambda_+ n_aug_);
    }

    is_initialized_ = true;
    return;
  }

  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) 
  {
    UpdateRadar(meas_package);
  } 
  else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) 
  {
    UpdateLidar(meas_package);
  }
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

  // Generate Sigma Points
  VectorXd X_aug = VectorXd(n_aug_);
  X_aug.head(n_x_) = x_;
  X_aug(n_x_) = 0;
  X_aug(n_x_+1) = 0;

  MatrixXd P_aug(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  MatrixXd Xsig_aug(n_aug_, 1 + 2 * n_aug_);
  Xsig_aug.col(0) = X_aug;

  MatrixXd L = P_aug.llt().matrixL();

  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)               = X_aug + sqrt(lambda_+ n_aug_) * L.col(i);
    Xsig_aug.col(n_aug_+i+1) = X_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // Predict Sigma Poiont
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yaw_rate = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);
    

    double px_p, py_p ;
    if (fabs(yaw_rate) > 0.001) { 
      px_p = px + v / yaw_rate * (sin(yaw + yaw_rate*delta_t) - sin(yaw));
      py_p = py + v / yaw_rate * (-cos(yaw + yaw_rate * delta_t) +cos(yaw));
    } else {  // drive straight
      px_p = px + cos(yaw) * v * delta_t;
      py_p = py + sin(yaw) * v * delta_t;
    }
    double v_p = v;
    double yaw_p = yaw + delta_t * yaw_rate;
    double yawd_p = yaw_rate;
    
    // Add noise
    px_p += 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
    py_p += 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
    v_p += delta_t * nu_a;
    yaw_p += 0.5 * delta_t * delta_t * nu_yawdd;
    yawd_p += nu_yawdd * delta_t;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // Predict Mean and Covariance
  VectorXd x_temp(5);
  x_temp.fill(0.0);

  for (int i = 0; i < 2*n_aug_+1; i++) {
    x_temp += weights_(i) * Xsig_pred_.col(i);
  }
  x_ = x_temp;

  MatrixXd P_temp(5, 5);
  P_temp.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = x_ - Xsig_pred_.col(i);
    x_diff(3) = Tools::ThetaValueCorrection(x_diff(3));
    P_temp += weights_(i) * (x_diff * x_diff.transpose());
  }
  P_ = P_temp;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  MatrixXd S(2, 2);
  MatrixXd K(5, 2);
  MatrixXd H(2, 5);
  H << 1, 0, 0, 0, 0,
           0, 1, 0, 0, 0;
  MatrixXd R(2, 2);
  R << std_laspx_ * std_laspx_, 0, 
          0, std_laspy_ * std_laspy_;

  VectorXd z = meas_package.raw_measurements_ - H * x_;
  S = H * P_ * H.transpose() + R;
  K = P_ * H.transpose() * S.inverse();

  MatrixXd I = MatrixXd::Identity(5,5);
  x_ = x_ + (K * z);
  P_ = (I - K * H) * P_;

  MatrixXd nis = z.transpose() * S.inverse() * z;
  cout <<  "L, " << nis(0, 0) << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  MatrixXd Zsig(3, 2 * n_aug_ + 1);
  Zsig = RadarHFunction(Xsig_pred_);

  // Z mean
  VectorXd Z(3);
  Z << 0, 0, 0;
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Z += weights_(i) * Zsig.col(i);
  }

  // S covariance
  MatrixXd S(3, 3);
  S.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Z - Zsig.col(i);
    z_diff(1) = Tools::ThetaValueCorrection(z_diff(1));

    S += weights_(i) * (z_diff * z_diff.transpose());
  }

  MatrixXd R(3, 3);
  R << std_radr_ * std_radr_, 0, 0, 
          0, std_radphi_ * std_radphi_, 0, 
          0, 0, std_radrd_ * std_radrd_;
  S += R;

  MatrixXd T(n_x_, 3);  // 5x3
  T.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = x_ -  Xsig_pred_.col(i);
    x_diff(3) = Tools::ThetaValueCorrection(x_diff(3));
    
    VectorXd z_diff = Z - Zsig.col(i);
    z_diff(1) = Tools::ThetaValueCorrection(z_diff(1));
    
    T += weights_(i) * (x_diff * z_diff.transpose());
  }

  MatrixXd K = T * S.inverse();  // 5x3
  VectorXd measurements = meas_package.raw_measurements_;

  MatrixXd z_diff = measurements - Z;
  z_diff(1) = Tools::ThetaValueCorrection(z_diff(1));

  // Update State
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  MatrixXd nis = z_diff.transpose() * S.inverse() * z_diff;
  cout << "R, " << nis(0, 0) << endl;
}

MatrixXd UKF::RadarHFunction(MatrixXd Xsig_pred) {
  MatrixXd Z(3, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_pred(0, i);
    double py = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double vx = cos(yaw) *v;
    double vy = sin(yaw) * v;

    double rho = sqrt(pow(px, 2) + pow(py, 2));
    double theta = atan2(py, px);
    double rho_dot = (px * vx + py * vy) / rho;

    Z(0, i) = rho;
    Z(1, i) = theta;
    Z(2, i) = rho_dot;
  }
  return Z;
}