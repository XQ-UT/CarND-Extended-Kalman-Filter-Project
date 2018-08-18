#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  Hj_ << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

  VectorXd x (4);
  x << 0, 0, 0, 0;

  MatrixXd P (4, 4);
  P << 0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0;

  MatrixXd F (4, 4);
  F << 1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;

  MatrixXd Q (4, 4);
  Q << 0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0;
  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float r = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);

      // TODO(xianlei): figure out initial speed.
      ekf_.x_ << r * cos(phi), r * sin(phi), 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);
      ekf_.x_ << px, py, 0, 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  long long delta_t = measurement_pack.timestamp_ - previous_timestamp_;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 2) = delta_t;
  ekf_.F_(1, 3) = delta_t;

  const long long noise_ax = 9;
  const long long noise_ay = 9;
  long long noise_ax_2 = noise_ax * noise_ax;
  long long noise_ay_2 = noise_ay * noise_ay;
  long long delta_t_2 = delta_t * delta_t;
  long long delta_t_3 = delta_t * delta_t_2;
  long long delta_t_4 = delta_t * delta_t_3;
  ekf_.Q_ << delta_t_4 * noise_ax_2 / 4, 0, delta_t_3 * noise_ax_2 / 2, 0,
             0, delta_t_4 * noise_ay_2 / 4, 0, delta_t_3 * noise_ay_2 / 2,
             delta_t_3 * noise_ax_2 / 2, 0, delta_t_2 * noise_ax_2, 0,
             0, delta_t_3 * noise_ay_2 / 2, 0, delta_t_2 * noise_ay_2;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
