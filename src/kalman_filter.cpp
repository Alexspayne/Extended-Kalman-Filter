#include "kalman_filter.h"
#include "tools.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_radar_in,
                        MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_laser_ = H_in;
  R_laser_ = R_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
        x_ = F_ * x_;
        MatrixXd Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
        VectorXd z_pred = H_laser_ * x_;
        VectorXd y = z - z_pred;
        MatrixXd Ht = H_laser_.transpose();
        MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
        MatrixXd Si = S.inverse();
        MatrixXd PHt = P_ * Ht;
        MatrixXd K = PHt * Si;

        // new estimate
        x_ = x_ + (K * y);
        int64 x_size = x_.size();
        MatrixXd I = MatrixXd::Identity(x_size, x_size);
        P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &Hj_) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // It's going to be mostly the same.
  // But this will be used for radar measurements.
  // So I'll use the Jacobian matrix.
  // Also, the measurement vector z has a different shape.
        VectorXd z_pred << Tools::h(x_);
        VectorXd y = z - z_pred;
        MatrixXd Ht = Hj_.transpose();
        MatrixXd S = Hj_ * P_ * Ht + R_radar_;
        MatrixXd Si = S.inverse();
        MatrixXd PHt = P_ * Ht;
        MatrixXd K = PHt * Si;

        // new estimate
        x_ = x_ + (K * y);
        int64 x_size = x_.size();
        MatrixXd I = MatrixXd::Identity(x_size, x_size);
        P_ = (I - K * Hj_) * P_;
}


