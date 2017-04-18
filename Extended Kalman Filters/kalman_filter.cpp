#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;


    // In FusionEKF, x_, Q_ and F_ are all updated.
    // P_ is updated here
    // R_ and H_ are never updated (but there's an R for RADAR and LIDAR)
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd S = H_ * P_ * Ht_ + R_;
    MatrixXd K = P_ * Ht_ * S.inverse();

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    double rho = sqrt(pow(px, 2.) +  pow(py, 2.));
    VectorXd z_pred = VectorXd(3);
    z_pred << rho,
            atan2(py, px),
            (px*vx + py* vy)/rho;

    MatrixXd Hj = tool.CalculateJacobian(x_);
    MatrixXd Hj_t = Hj.transpose();
    VectorXd y = z - z_pred;
    double theta = y(1);
    if (theta > PI) {
        while (theta > PI) {
            theta -= twoPI;
        }
        y(1) = theta;
    } else if (theta < negative_PI) {
        while (theta < negative_PI) {
            theta += twoPI;
        }
        y(1) = theta;
    }
    std::cout << "y" <<  y;
    MatrixXd S = Hj * P_ * Hj_t + R_radar;
    MatrixXd K = P_ * Hj_t * S.inverse();

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj) * P_;

}
