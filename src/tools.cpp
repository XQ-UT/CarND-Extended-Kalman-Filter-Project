#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  for(int i = 0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    VectorXd residual_2 = residual.array() * residual.array();
    rmse += residual_2;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj (3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px_py_square = px * px + py * py;
  float px_py_square_root = sqrt(px_py_square);
  float px_py_3_over_2 = px_py_square * px_py_square_root;

  if (fabs(px_py_square) < 0.0001){
    cout << "Divided by 0 occurs while calculating Jacobian Matrix" << endl;
    return Hj;
  }

  Hj << (px / px_py_square_root), (py / px_py_square_root), 0, 0,
        (- py / px_py_square), (px / px_py_square), 0, 0,
        py * (vx * py - vy * px) / px_py_3_over_2, px * (vy * px - vx * py) / px_py_3_over_2, px / px_py_square, py / px_py_square;

  return Hj;
}
