#ifndef _EKF_H
#define _EKF_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

typedef Eigen::Matrix<float,2,4> Matrix24f;
typedef Eigen::Matrix<float,4,2> Matrix42f;

/**
 * Extended Kalman Filter Implementation
 */
class EKF
{
public:
  /** 
   * Default constructor. 
   */
  EKF();


  void runKF(Eigen::Vector4f& mu_hat, Eigen::Matrix4f& sig_hat,Eigen::Vector4f& ut,
                    Eigen::Vector2f& zt, Eigen::Matrix4f& A, Eigen::Matrix4f& B, Matrix24f C,
                    Eigen::Matrix2f& Q, Eigen::Matrix4f& R, bool useMeas);
  void predictionStep(Eigen::Vector4f& mu_hat,Eigen::Matrix4f& sig_hat,Eigen::Vector4f& ut,
                      Eigen::Matrix4f& A, Eigen::Matrix4f& B, Eigen::Matrix4f& R,
                      Eigen::Vector4f& mu_bar, Eigen::Matrix4f& sig_bar);

  void updateStep(Eigen::Vector4f& mu_bar, Eigen::Matrix4f& sig_bar, Matrix24f C, 
                  Eigen::Vector2f& zt, Eigen::Vector2f& z_bar, Eigen::Matrix2f& Q,
                  Eigen::Vector4f& mu_hat, Eigen::Matrix4f& sig_hat);

// private:


};

#endif