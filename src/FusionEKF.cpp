#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
  
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;
  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1000, 0,
           0, 0, 0, 1000;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  //printf("FusionEKF::ProcessMeasurement\n");
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
      //cout << "result:" << measurement_pack.raw_measurements_(0) << endl;
	  ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Initialize state.
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
      //cout << "result:" << measurement_pack.raw_measurements_(2) << endl;
      float ro = measurement_pack.raw_measurements_(0);
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
      float phi = measurement_pack.raw_measurements_(1);
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
      float ro_dot = measurement_pack.raw_measurements_(2);
      
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
      ekf_.x_(0) = ro * cos(phi);
      ekf_.x_(1) = ro * sin(phi);
      ekf_.x_(2) = ro_dot * cos(phi);
      ekf_.x_(3) = ro_dot * sin(phi);
      
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
      /*if ( ekf_.x_(0) < 0.0001 ) {
        ekf_.x_(0) = 0.0001;
      }
      
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
      if ( ekf_.x_(1) < 0.0001 ) {
        ekf_.x_(1) = 0.0001;
      }*/
      //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
    }
	
    //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    //printf("FusionEKF::ProcessMeasurement [%d]\n", __LINE__);
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;

  // State transition matrix update
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // set acceleration of noise components
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
	         0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
	         dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
 	         0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    
  	//std::cout << "RADAR:: ekf_.x_:" << ekf_.x_ << std::endl;
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	//std::cout << "RADAR:: ekf_.H_:" << ekf_.H_ << std::endl;
  	//std::cout << "ekf_.H_:" << ekf_.H_ << std::endl;
    
  	ekf_.R_ = R_radar_;
    
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
  	//std::cout << "Laser::  ekf_.H_:" << ekf_.H_ << std::endl;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
