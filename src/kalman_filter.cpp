#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  //printf("Kalman Filter Init\n");
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  //printf("KalmanFilter::Predict\n");
  x_ = F_ * x_;
  //std::cout << "x_:" << x_ << std::endl;
  MatrixXd Ft = F_.transpose();
  //std::cout << "P_:" << P_ << std::endl;
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  //printf("KalmanFilter::Update\n");
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  
  //std::cout << "x_:" << x_ << std::endl;
  x_ = x_ + (K * y);
  //std::cout << "x_:" << x_ << std::endl;
  
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  //std::cout << "P_:" << P_ << std::endl;
  P_ = (I - K * H_) * P_;
  //std::cout << "P_:" << P_ << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //printf("KalmanFilter::Update EKF\n");
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  
  VectorXd h = VectorXd(3);
  VectorXd y;

  double rho = sqrt(px*px + py*py);
  double theta = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;
  /*double rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (px*vx + py*vy) / rho;
  }*/
  
  h << rho, theta, rho_dot;
  y = z - h;
  while ( y(1) > M_PI || y(1) < -M_PI ) 
  {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }
  //std::cout << "H_:" << H_ << std::endl;
  MatrixXd Ht = H_.transpose();
  //std::cout << "Ht:" << Ht << std::endl;
  //std::cout << "R_:" << R_ << std::endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  //std::cout << "S:" << S << std::endl;
  MatrixXd Si = S.inverse();
  //std::cout << "Si:" << Si << std::endl;
  //std::cout << "P_:" << P_ << std::endl;
  MatrixXd K =  P_ * Ht * Si;
  //std::cout << "K:" << K << std::endl;
 
  
  //std::cout << "x_:" << x_ << std::endl;
  //std::cout << "K:" << K << std::endl;
  //std::cout << "y:" << y << std::endl;
  x_ = x_ + (K * y);
  /*if (abs(x_(0)) > 0.11 ||
      abs(x_(1)) > 0.11 ||
      abs(x_(2)) > 0.52 ||
      abs(x_(3)) > 0.52)
    printf("####################\n");*/
  
  //std::cout << "x_:" << x_ << std::endl;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  //std::cout << "P_:" << P_ << std::endl;
  //std::cout << "I:" << I << std::endl;
  //std::cout << "K:" << K << std::endl;
  //std::cout << "H_:" << H_ << std::endl;
  P_ = (I - K * H_) * P_;
  //std::cout << "P_:" << P_ << std::endl;
}
