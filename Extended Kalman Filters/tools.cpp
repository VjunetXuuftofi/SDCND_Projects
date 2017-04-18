#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);


    //check division by zero
    double vx = x_state(2);
    double vy = x_state(3);

    double px_2 = pow(px, 2.);
    double py_2 = pow(py, 2.);

    double sum_px_2_py_2 = px_2 + py_2;

    double sqrt_sum = sqrt(sum_px_2_py_2);

    double px_over_sqrt_sum = px / sqrt_sum;

    double py_over_sqrt_sum = py / sqrt_sum;

    double pow_3_2_sum = pow(sum_px_2_py_2, 3./2);

    Hj << px_over_sqrt_sum, py_over_sqrt_sum, 0, 0,
            - (py / (sum_px_2_py_2)), (px / (sum_px_2_py_2)), 0, 0,
            py*(vx*py - vy*px) / pow_3_2_sum, px*(vy*px - vx*py) / pow_3_2_sum,
            px_over_sqrt_sum, py_over_sqrt_sum;
    //compute the Jacobian matrix

    return Hj;
}
