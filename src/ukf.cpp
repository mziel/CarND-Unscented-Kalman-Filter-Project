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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3; //0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03; //0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3; //0.1;

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  //create augmented mean vector
  x_aug_ = VectorXd(n_aug_);

  // augmented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.fill(0.0);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // time when the state is true, in us
  time_us_ = 1000;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Previous timestamp
  previous_timestamp_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  cout << "Processing measurment" << endl;
  bool processCurrentRadar = (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_);
  bool processCurrentLaser = (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_);
  bool processCurent = processCurrentRadar || processCurrentLaser;
  if (processCurent) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "Radar" << endl;

        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        x_.head(4) = tools.PolarToCarthesian(meas_package.raw_measurements_);
        x_(2) = 0.0; // radar measurement does not contain enough information to determine the state variable velocities
        x_(3) = 0.0; // radar measurement does not contain enough information to determine the yaw
        x_(4) = 0.0; // radar measurement does not contain enough information to determine the yaw rate
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      cout << "Lidar" << endl;

        /**
        Initialize state
        */
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0.0, 0.0, 0.0;
      }

      // Create the covariance matrix
      float init_variance = 1;
      P_ << init_variance, 0, 0, 0, 0,
               0, init_variance, 0, 0, 0,
               0, 0, init_variance, 0, 0,
               0, 0, 0, init_variance, 0,
               0, 0, 0, 0, init_variance;

      previous_timestamp_ = meas_package.timestamp_;

      // done initializing, no need to predict or update
      is_initialized_ = true;
      return;
    } else {

      /*****************************************************************************
       *  Prediction
       ****************************************************************************/
    
      // Update the state transition matrix F according to the new elapsed time
      double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
      previous_timestamp_ = meas_package.timestamp_;
      Prediction(delta_t);
    
      /*****************************************************************************
       *  Update
       ****************************************************************************/
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
      } else {
        UpdateLidar(meas_package);
      }
    
      // print the output
      cout << "x_ = " << x_ << endl;
      cout << "P_ = " << P_ << endl;
      cout << "_____________________________ " << endl;
    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    cout << "Prediction step" << endl;
    AugmentedSigmaPoints();
    SigmaPointPrediction(delta_t);
    PredictMeanAndCovariance();
 }

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  cout << "Updating lidar" << endl;  
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  cout << "Updating radar" << endl;  
  /**
  TODO:
  You'll also need to calculate the radar NIS.
  */
    int n_z = 3;
    VectorXd z_out = VectorXd(n_z);
    MatrixXd S_out = MatrixXd(n_z, n_z);
    MatrixXd Z_sig_out = MatrixXd(n_z, 2 * n_aug_ + 1);
    PredictRadarMeasurement(&z_out, &S_out, &Z_sig_out, n_z);
    UpdateRadarState(z_out, S_out, Z_sig_out, n_z, meas_package.raw_measurements_);        
}


void UKF::AugmentedSigmaPoints() {
  cout << "Calculating sigma points" << endl;   

  //fill augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //fill augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i + 1)          = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  cout << "Xsig_aug_ = " << Xsig_aug_ << endl;     
}

void UKF::SigmaPointPrediction(double delta_t) {
  cout << "Predicting with sigma points" << endl;   

  // compute predictions
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      // Setup initial state
      VectorXd x_k = Xsig_aug_.col(i); 
      float p_x = x_k(0);
      float p_y = x_k(1);
      float v = x_k(2);
      float psi = x_k(3);
      float psi_dot = x_k(4);
      float nu_a = x_k(5);
      float nu_psi_ddot = x_k(6);
      
      // Setup stochastic part
      VectorXd stochastic = VectorXd(n_x_);
      stochastic(0) = (1/2) * delta_t * delta_t * cos(psi) * nu_a;
      stochastic(1) = (1/2) * delta_t * delta_t * sin(psi) * nu_a;
      stochastic(2) = delta_t * nu_a;
      stochastic(3) = (1/2) * delta_t * delta_t * nu_psi_ddot;
      stochastic(4) = delta_t * nu_psi_ddot;
    
      //predict deterministic
      VectorXd deterministic = VectorXd(n_x_);
      if (fabs(psi_dot) < 0.0001) {
          deterministic(0) = v * cos(psi) * delta_t;
          deterministic(1) = v * sin(psi) * delta_t;
          deterministic(2) = 0;
          deterministic(3) = 0;
          deterministic(4) = 0;
      } else {
          deterministic(0) = v / psi_dot * (sin(psi + psi_dot * delta_t) - sin(psi));
          deterministic(1) = v / psi_dot * (-cos(psi + psi_dot * delta_t) - cos(psi));
          deterministic(2) = 0;
          deterministic(3) = psi_dot * delta_t;
          deterministic(4) = 0;
      }
     
      //set output state
      Xsig_pred_.col(i) = x_k.head(n_x_) + deterministic + stochastic;
  }
  cout << "Xsig_pred_ = " << Xsig_pred_ << endl;       
}

void UKF::PredictMeanAndCovariance() {
  cout << "Predicting mean and covariance" << endl;   

  //set weights
  weights_.fill( 1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  
  //predict state mean
  x_ = Xsig_pred_ * weights_;
  
  //predict state covariance matrix
  MatrixXd X_temp = Xsig_pred_;
  X_temp.colwise() -= x_;
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = X_temp.col(i);
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }  
  cout << "x_ = " << x_ << endl;       
  cout << "P_ = " << P_ << endl;       
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Z_sig_out, int n_z) {
  cout << "Predicting radar measurement" << endl;   

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v  = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                            //r
    Zsig(1, i) = atan2(p_y, p_x);                                        //phi
    Zsig(2, i) = (p_x * v1 + p_y * v2 ) / sqrt(p_x * p_x + p_y * p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Z_sig_out = Zsig;
  cout << "z_pred = " << z_pred << endl;       
  cout << "S = " << S << endl;       
  cout << "Z_sig_out = " << Zsig << endl;       
}

void UKF::UpdateRadarState(VectorXd z_pred, MatrixXd S_pred, MatrixXd Z_sig, int n_z, VectorXd z) {
  cout << "Updating state for radar" << endl;   

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_pred.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_pred*K.transpose();
  cout << "x_ = " << x_ << endl;       
  cout << "P_ = " << P_ << endl;       
}
