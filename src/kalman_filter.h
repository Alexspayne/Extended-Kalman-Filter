#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
 public:
  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // state transistion matrix
  MatrixXd F_;

  // process covariance matrix
  MatrixXd Q_;

  // measurement matrix
  MatrixXd H_laser_;

  // measurement covariance matrix for laser
  MatrixXd R_laser_;

  // measurement covariance matrix for laser
  MatrixXd R_radar_;

  /**
   * Constructor
   */
  KalmanFilter();
  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
            MatrixXd &H_in, MatrixXd &R_in, MatrixXd &R_radar_in,
            MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z, const MatrixXd &Hj_);

  // tools is used so to access the h(x) function.
  Tools tools;
};

#endif /* KALMAN_FILTER_H_ */
