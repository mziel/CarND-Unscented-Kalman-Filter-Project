#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored
  use_laser_ = true;

  // if this is false, radar measurements will be ignored
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  // Create the covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

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

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // create augmented mean vector
  x_aug_ = VectorXd(n_aug_);

  // augmented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.fill(0.0);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // time when the state is true, in us
  time_us_ = 0;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Previous timestamp
  previous_timestamp_ = 0;

  // Covariance matrix for laser measurement noise
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  // Covariance matrix for radar measurement noise
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0,
      0, std_radrd_ * std_radrd_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &meas_package) {
  // cout << "Processing measurment" << endl;
  bool processCurrentRadar =
      (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_);
  bool processCurrentLaser =
      (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_);
  bool processCurent = processCurrentRadar || processCurrentLaser;
  if (processCurent) {
    // Initialization
    if (!is_initialized_) {
      InitializeState(meas_package);
    } else {
      // Prediction
      Prediction(meas_package);

      // Update
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
      } else {
        UpdateLidar(meas_package);
      }
      // print the output
      // cout << "x_ = " << x_ << endl;
      // cout << "P_ = " << P_ << endl;
      // cout << "_____________________________ " << endl;
    }
  }
}

/**
 * Initialize state upon receiving a first measurement
 * @param meas_package The latest measurement data of either radar or laser
 */
void UKF::InitializeState(const MeasurementPackage &meas_package) {
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // cout << "Radar" << endl;
    // Convert radar from polar to cartesian coordinates and initialize state.
    x_.head(4) = tools.PolarToCarthesian(meas_package.raw_measurements_);
    x_(2) = 0.0; // radar measurement does not contain enough information to
                 // determine the state variable velocities
    x_(3) = 0.0; // radar measurement does not contain enough information to
                 // determine the yaw
    x_(4) = 0.0; // radar measurement does not contain enough information to
                 // determine the yaw rate
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // cout << "Lidar" << endl;
    // Initialize state
    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1],
        0.0, 0.0, 0.0;
  }

  previous_timestamp_ = meas_package.timestamp_;
  is_initialized_ = true;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {MeasurementPackage} meas_package
 * measurement and this one.
 */
void UKF::Prediction(const MeasurementPackage &meas_package) {
  // cout << "Prediction step" << endl;
  double delta_t = (meas_package.timestamp_ - previous_timestamp_) /
                   1000000.0; // dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  AugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage &meas_package) {
  // cout << "Updating lidar" << endl;
  /**
  TODO:
  You'll also need to calculate the lidar NIS.
  */
  int n_z = 2;
  VectorXd z_out = VectorXd(n_z);
  MatrixXd S_out = MatrixXd(n_z, n_z);
  MatrixXd Z_sig_out = MatrixXd(n_z, 2 * n_aug_ + 1);
  PredictLidarMeasurement(&z_out, &S_out, &Z_sig_out, n_z);
  UpdateState(z_out, S_out, Z_sig_out, n_z, meas_package.raw_measurements_);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage &meas_package) {
  // cout << "Updating radar" << endl;
  /**
  TODO:
  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;
  VectorXd z_out = VectorXd(n_z);
  MatrixXd S_out = MatrixXd(n_z, n_z);
  MatrixXd Z_sig_out = MatrixXd(n_z, 2 * n_aug_ + 1);
  PredictRadarMeasurement(&z_out, &S_out, &Z_sig_out, n_z);
  UpdateState(z_out, S_out, Z_sig_out, n_z, meas_package.raw_measurements_);
}

/**
 * Creates sigma points in the augmented space (incl. noise)
 */
void UKF::AugmentedSigmaPoints() {
  // cout << "Calculating sigma points" << endl;

  // fill augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // fill augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  // cout << "Xsig_aug_ = " << Xsig_aug_ << endl;
}

/**
 * Predicts sigma points in the process function
 * @param delta_t Time difference between measurements
 */
void UKF::SigmaPointPrediction(const double delta_t) {
  // cout << "Predicting with sigma points" << endl;

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
  // cout << "Xsig_pred_ = " << Xsig_pred_ << endl;
}

/**
 * Provides a prediction for state mean and covariance
 */
void UKF::PredictMeanAndCovariance() {
  // cout << "Predicting mean and covariance" << endl;
  // set weights
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // predicted state mean
  x_ = Xsig_pred_ * weights_;

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    x_diff(3) = tools.NormalizeAtanDiff(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  // cout << "x_ = " << x_ << endl;
  // cout << "P_ = " << P_ << endl;
}

/**
 * Predicts mean and covariance after passing through measurement function for
 * radar
 */
void UKF::PredictRadarMeasurement(VectorXd *z_out, MatrixXd *S_out,
                                  MatrixXd *Z_sig_out, const int n_z) {
  // cout << "Predicting radar measurement" << endl;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                         // r
    Zsig(1, i) = atan2(p_y, p_x);                                     // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); // r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = tools.NormalizeAtanDiff(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  S = S + R_radar_;

  // write result
  *z_out = z_pred;
  *S_out = S;
  *Z_sig_out = Zsig;
  // cout << "z_pred = " << z_pred << endl;
  // cout << "S = " << S << endl;
  // cout << "Z_sig_out = " << Zsig << endl;
}

/**
 * Predicts mean and covariance after passing through measurement function for
 * lidar
 */
void UKF::PredictLidarMeasurement(VectorXd *z_out, MatrixXd *S_out,
                                  MatrixXd *Z_sig_out, const int n_z) {
  // cout << "Predicting lidar measurement" << endl;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  S = S + R_laser_;

  // write result
  *z_out = z_pred;
  *S_out = S;
  *Z_sig_out = Zsig;
  // cout << "z_pred = " << z_pred << endl;
  // cout << "S = " << S << endl;
  // cout << "Z_sig_out = " << Zsig << endl;
}

/**
 * Performs state update based on the measurement and prediction
 */
void UKF::UpdateState(const VectorXd &z_pred, const MatrixXd &S_pred,
                      const MatrixXd &Z_sig, const int n_z, const VectorXd &z) {
  // cout << "Updating state" << endl;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // 2n+1 simga points

    // residual
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    // angle normalization
    z_diff(1) = tools.NormalizeAtanDiff(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    x_diff(3) = tools.NormalizeAtanDiff(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S_pred.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  if (n_z == 3) // Radar only
  {
    // angle normalization
    z_diff(1) = tools.NormalizeAtanDiff(z_diff(1));
  }
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_pred * K.transpose();
  // cout << "x_ = " << x_ << endl;
  // cout << "P_ = " << P_ << endl;
}
