#ifndef _EKF_H
#define _EKF_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include "MatrixDefinitions.h"

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


  void runKF(VectorMuf& mu_hat, MatrixSigf& sig_hat,VectorUtf& ut,
             VectorZtf& zt, MatrixAf& A, MatrixBf& B, MatrixCf C,
             MatrixQf& Q, MatrixRf& R, bool useMeas);
  void predictionStep(VectorMuf& mu_hat,MatrixSigf& sig_hat, VectorUtf& ut,
                      MatrixAf& A, MatrixBf& B, MatrixRf& R,
                      VectorMuf& mu_bar, MatrixSigf& sig_bar);

  void updateStep(VectorMuf& mu_bar, MatrixSigf& sig_bar, MatrixCf C, 
                  VectorZtf& zt, VectorZtf& z_bar, MatrixQf& Q,
                  VectorMuf& mu_hat, MatrixSigf& sig_hat);

// private:


};

#endif