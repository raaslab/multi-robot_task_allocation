/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "max_min_lp_simulation/kalman.hpp"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), B(B), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y, const Eigen::VectorXd& u) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  // std::cout<<"x_hat = "<<x_hat[0]<<", "<<x_hat[1]<<std::endl;
  x_hat_new = A * x_hat + B * u;
  // std::cout<<"x_hat_new = "<<x_hat_new[0]<<", "<<x_hat_new[1]<<std::endl;
  P = A*P*A.transpose() + Q;
  // std::cout<<"P = "<<P<<std::endl;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  // std::cout<<"K = "<<K<<std::endl;
  x_hat_new += K * (y - C*x_hat_new);
  // std::cout<<"x_hat_new = "<<x_hat_new[0]<<", "<<x_hat_new[1]<<std::endl;
  P = (I - K*C)*P;
  // std::cout<<"P = "<<P<<std::endl;
  x_hat = x_hat_new;
  // std::cout<<"x_hat = "<<x_hat[0]<<", "<<x_hat[1]<<std::endl;

  t += dt;
}