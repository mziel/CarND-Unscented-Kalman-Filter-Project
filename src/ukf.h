#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "tools.h"
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

  ///* if this is false, laser measurements will be ignored
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  //* augmented state vector;
  VectorXd x_aug_;

  ///* augmented sigma points matrix
  MatrixXd Xsig_aug_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

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

  ///* Previous timestamp
  long long previous_timestamp_;

  ///* Helper object
  Tools tools;

  /**
   * Constructor
   */
  UKF();

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
   * Initialize state upon receiving a first measurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void InitializeState(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param meas_package Measurement to extract delta between k and k+1 
   */
  void Prediction(MeasurementPackage meas_package);

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

  /**
   * Creates sigma points in the augmented space (incl. noise)
   */
  void AugmentedSigmaPoints();

  /**
   * Predicts sigma points in the process function
   * @param delta_t Time difference between measurements
   */
  void SigmaPointPrediction(double delta_t);

  /**
   * Provides a prediction for state mean and covariance
   */
  void PredictMeanAndCovariance();

  /**
   * Predicts mean and covariance after passing through measurement function for radar
   */
  void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Z_sig_out, int n_z);

  /**
   * Predicts mean and covariance after passing through measurement function for lidar
   */
  void PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Z_sig_out, int n_z);

  /**
   * Performs state update based on the measurement and prediction 
   */
  void UpdateState(VectorXd z_pred, MatrixXd S_pred, MatrixXd Z_sig, int n_z, VectorXd z);
};

#endif /* UKF_H */
