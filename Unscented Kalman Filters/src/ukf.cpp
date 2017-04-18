#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.25;
    std_a_squared_ = std_a_*std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

    std_yawdd_squared_ = std_yawdd_*std_yawdd_;



  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
    std_laspx_squared = std_laspx_*std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
    std_laspy_squared = std_laspy_*std_laspy_;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

    std_radr_squared_ = std_radr_*std_radr_;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

    std_radphi_squared_ = std_radphi_*std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

    std_radrd_squared_ = std_radrd_*std_radrd_;

    n_x_ = 5;
    n_aug_ = 7;
    n_radar_ = 3;
    n_lidar_ = 2;
    lambda_ = 3-n_aug_;
    sqrt_3 = sqrt(3); // sqrt(sigma + n_aug);
    n_sigma = 2*n_aug_+1;
    is_initialized_ = false;

    twoPI = 2. * M_PI;
    negative_PI = -1*M_PI;
    Xsig_pred_ = MatrixXd(n_x_, n_sigma);
    R_lidar_ = MatrixXd(n_lidar_, n_lidar_);
    R_lidar_ << std_laspx_squared, 0,
            0, std_laspy_squared;

    R_radar_ = MatrixXd(n_radar_, n_radar_);
    R_radar_ << std_radr_squared_, 0, 0,
            0, std_radphi_squared_, 0,
            0, 0, std_radrd_squared_;





    weights_ = VectorXd(n_sigma);
    weights_(0) = lambda_/3; // 3 here is n_aug + lambda
    double weight = 0.5/3;
    for (int i=1; i < n_sigma; i++) {
        weights_(i) = weight;
    }
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
    VectorXd raw_measurements = meas_package.raw_measurements_;

    if (!is_initialized_) {

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = raw_measurements(0);
            double phi = raw_measurements(1);
            x_ << rho*cos(phi), rho*sin(phi), 0, 0, 0;
        }
        else {
            x_ << raw_measurements(0), raw_measurements(1), 0, 0, 0;
        }
        if (x_(0) == 0 && x_(1) == 0) { //ignore nonsense measurments. Could initialize with small values, but doesn't
            //                            make much sense.
            return;
        }

                  // done initializing, no need to predict or update
        previous_timestamp_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    bool is_laser = meas_package.sensor_type_ == MeasurementPackage::LASER;
    if (is_laser && !use_laser_) {
        return;
    }
    if (!is_laser && !use_radar_) {
        return;
    }
    double delta_t = (meas_package.timestamp_ - previous_timestamp_) /1000000.0;
    // 3.95712
    MatrixXd old_P = P_;
    VectorXd old_X = x_;
    Prediction(delta_t);

    //Disregard predictions where x or y are zero.
    if (x_(0) == 0 || x_(1) == 0) {
        return;
    }
    previous_timestamp_ = meas_package.timestamp_;

    if (is_laser) {
        UpdateLidar(meas_package);

    } else {
        UpdateRadar(meas_package);
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
    //Initialize augmented matrices
    VectorXd x_aug = VectorXd(n_aug_);
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma);

    // create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;


    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = std_a_squared_;
    P_aug(6, 6) = std_yawdd_squared_;


    MatrixXd L = P_aug.llt().matrixL();


    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++) {
        int i_plus_one = i+1;
        VectorXd L_col_i = L.col(i);
        Xsig_aug.col(i_plus_one) = x_aug + sqrt_3 * L_col_i;
        Xsig_aug.col(i_plus_one+n_aug_) = x_aug - sqrt_3 * L_col_i;
    }



    //predict sigma points
    for (int i = 0; i < n_sigma; i++) {
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        //predicted state values
        double px_p, py_p;


        double cos_yaw = cos(yaw);
        double sin_yaw = sin(yaw);
        if (fabs(yawd) > 0.001) {
            double v_over_yawd = v/yawd;
            double yaw_plus_yawd_times_delta_t = yaw + yawd*delta_t;
            px_p = p_x + v_over_yawd * ( sin (yaw_plus_yawd_times_delta_t) - sin_yaw);
            py_p = p_y + v_over_yawd * ( cos_yaw - cos(yaw_plus_yawd_times_delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos_yaw;
            py_p = p_y + v*delta_t*sin_yaw;
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        double delta_t_squared = delta_t*delta_t;
        double half_nu_a_times_delta_t_squared = 0.5*delta_t_squared*nu_a;

        //add noise
        px_p = px_p + half_nu_a_times_delta_t_squared * cos_yaw;
        py_p = py_p + half_nu_a_times_delta_t_squared * sin_yaw;
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t_squared;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }


    //predict state mean
    x_.fill(0.0);
    for (int i = 0; i < n_sigma; i++) {
        double this_weight = weights_(i);
        for (int j = 0; j < n_x_; j++) {
            x_(j) += this_weight*Xsig_pred_(j, i);
        }
    }


    // Update P
    P_.fill(0.0);
    for (int i = 0; i < n_sigma; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        while (x_diff(3) > M_PI) x_diff(3) -= twoPI;
        while (x_diff(3) < negative_PI) x_diff(3) += twoPI;
        P_ += weights_(i)*x_diff*x_diff.transpose();
    }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    MatrixXd Zsig = MatrixXd(n_lidar_, n_sigma);
    VectorXd z_pred = VectorXd(n_lidar_);
    z_pred.fill(0.0);
    MatrixXd S = MatrixXd(n_lidar_, n_lidar_);
    S.fill(0.0);

    //transform sigma points into measurement space
    for (int i = 0; i < n_sigma; i++) {
        VectorXd x_k_plus_1 = Xsig_pred_.col(i);
        Zsig.col(i) << x_k_plus_1(0),
                       x_k_plus_1(1);
    }

    //calculate mean predicted measurement
    for (int i = 0; i < n_sigma; i++) {
        for (int dim = 0; dim < n_lidar_; dim++) {
            z_pred(dim) += weights_(i)*Zsig(dim, i);
        }
    }




    //Calculate measurement covariance matrix S
    for (int dim = 0; dim < n_sigma; dim++) {
        VectorXd z_diff = Zsig.col(dim)-z_pred;
        S += weights_(dim)*z_diff*z_diff.transpose();
    }
    S += R_lidar_;



    MatrixXd Tc = MatrixXd(n_x_, n_lidar_);
    Tc.fill(0.0);

    for (int i = 0; i < n_sigma; i++) {
        VectorXd z_diff = Zsig.col(i)-z_pred;

        VectorXd x_diff = Xsig_pred_.col(i)-x_;
        while (x_diff(3) < negative_PI) x_diff(3) += twoPI;
        while (x_diff(3) > M_PI) x_diff(3) -= twoPI;

        Tc += weights_(i)*x_diff*z_diff.transpose();
    }

    MatrixXd S_inverse = S.inverse();
    //Kalman gain
    MatrixXd K = Tc*S_inverse;




    //update state mean and covariance matrix
    VectorXd z = meas_package.raw_measurements_;
    VectorXd error = z-z_pred;
    NIS_laser_ = error.transpose()*S_inverse*error;


    x_ = x_ + K*(error);

    P_ = P_ - K*S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  MatrixXd Zsig = MatrixXd(n_radar_, n_sigma);
    VectorXd z_pred = VectorXd(n_radar_);
    z_pred.fill(0.0);
    MatrixXd S = MatrixXd(n_radar_, n_radar_);
    S.fill(0.0);

    //transform sigma points into measurement space
    for (int i = 0; i < n_sigma; i++) {
        VectorXd x_k_plus_1 = Xsig_pred_.col(i);
        double px = x_k_plus_1(0);
        double py = x_k_plus_1(1);
        double nu = x_k_plus_1(2);
        double rho = x_k_plus_1(3);
        double pythag = sqrt(pow(px, 2) + pow(py, 2));
        if (fabs(pythag) < 0.001) {
            pythag = 0.001;
        }
        Zsig.col(i) << pythag,
                      atan(py/px),
                      (px*cos(rho)*nu + py*sin(rho)*nu)/pythag;
    }

    //calculate mean predicted measurement
    for (int i = 0; i < n_sigma; i++) {
        for (int dim = 0; dim < n_radar_; dim++) {
            z_pred(dim) += weights_(i)*Zsig(dim, i);
        }
    }


    //Calculate measurement covariance matrix S HERE
    for (int dim = 0; dim < n_sigma; dim++) {
        VectorXd z_diff = Zsig.col(dim)-z_pred;
        while (z_diff(1) < negative_PI) z_diff(1) += twoPI;
        while (z_diff(1) > M_PI) z_diff(1) -= twoPI;
        S += weights_(dim)*z_diff*z_diff.transpose();
    }
    S += R_radar_;

    MatrixXd Tc = MatrixXd(n_x_, n_radar_);
    Tc.fill(0.0);

    for (int i = 0; i < n_sigma; i++) {
        VectorXd z_diff = Zsig.col(i)-z_pred;
        while (z_diff(1) < negative_PI) z_diff(1) += twoPI;
        while (z_diff(1) > M_PI) z_diff(1) -= twoPI;

        VectorXd x_diff = Xsig_pred_.col(i)-x_;
        while (x_diff(3) < negative_PI) x_diff(3) += twoPI;
        while (x_diff(3) > M_PI) x_diff(3) -= twoPI;

        Tc += weights_(i)*x_diff*z_diff.transpose();
    }

    MatrixXd S_inverse = S.inverse();
    //Kalman gain
    MatrixXd K = Tc*S_inverse;

    //update state mean and covariance matrix
    VectorXd z = meas_package.raw_measurements_;
    VectorXd error = z - z_pred;
    while (error(1) < negative_PI) error(1) += twoPI;
    while (error(1) > M_PI) error(1) -= twoPI;

    NIS_radar_ = error.transpose()*S_inverse*error;


    x_ = x_ + K*(error);

    P_ = P_ - K*S*K.transpose();
}
