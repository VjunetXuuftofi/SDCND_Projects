#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

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

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

    double std_a_squared_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;
    double std_yawdd_squared_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;
    double std_laspx_squared;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;
    double std_laspy_squared;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

    double std_radr_squared_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

    double std_radphi_squared_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;
    double std_radrd_squared_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

    ///* Radar state dimension
    int n_radar_;

    ///* Lidar state dimension
    int n_lidar_;

  ///* Sigma point spreading parameter
  double lambda_;

    int n_sigma;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

    long previous_timestamp_;
    double sqrt_3;
    double twoPI;
    double negative_PI;

    MatrixXd R_lidar_;
    MatrixXd R_radar_;


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
