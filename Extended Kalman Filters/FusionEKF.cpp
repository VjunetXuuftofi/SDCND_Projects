#include "FusionEKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    /**
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */
    ekf_.x_ = VectorXd(4);
    ekf_.Q_ = MatrixXd(4, 4);
    //create a 4D state vector, we don't know yet the values of the x state
    ekf_.x_ = VectorXd(4);

    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


    //measurement covariance
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ << 0.0225, 0,
            0, 0.0225;

    ekf_.R_radar = R_radar_;

    //measurement matrix
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;
    ekf_.Ht_ = ekf_.H_.transpose();

    //the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;


    //recover state parameters


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    VectorXd raw_measurements = measurement_pack.raw_measurements_;

    if (!is_initialized_) {
        /**
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "EKF: " << endl;

        //set the acceleration noise components

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = raw_measurements(0);
            double phi = raw_measurements(1);
            ekf_.x_ << rho*cos(phi), rho*sin(phi), 0, 0;


        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_ << raw_measurements(0), raw_measurements(1), 0, 0;
        }

        // done initializing, no need to predict or update
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    double dt_4_over_4 = dt_4 / 4.;
    double dt_3_over_2 = dt_3 / 2.;
    ekf_.Q_ <<
            dt_4_over_4*noise_ax, 0, dt_3_over_2*noise_ax, 0,
            0, dt_4_over_4*noise_ay, 0, dt_3_over_2*noise_ay,
            dt_3_over_2*noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3_over_2*noise_ay, 0, dt_2*noise_ay;
    VectorXd old_x_ = ekf_.x_;
    ekf_.Predict();



    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        if (ekf_.x_(0) == 0 && ekf_.x_(1) == 0) {
            ekf_.x_ = old_x_;
            return;
        }
        ekf_.UpdateEKF(raw_measurements);
        // Radar updates
    } else {
        ekf_.Update(raw_measurements);
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
