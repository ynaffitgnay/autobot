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
  EKF();

  void runEKF(VectorMuf& mu_hat, MatrixSigf& sig_hat,VectorUtf& ut, VectorZtf& zt,
              std::function <void(VectorMuf&, VectorUtf&, VectorMuf&)> calcMuBar,
              std::function <void(VectorMuf&, MatrixAGf&, MatrixCHf&)> calcGH,
              std::function <void(VectorMuf&, VectorZtf&)> calcMeasPred,
              MatrixQf& Q, MatrixRf& R, bool useMeas);

  void predictionStep(VectorMuf& mu_hat,MatrixSigf& sig_hat, VectorUtf& ut,
                      MatrixAGf& A_or_G, MatrixCHf& C_or_H, MatrixRf& R,
                      VectorMuf& mu_bar, MatrixSigf& sig_bar,
                      std::function <void(VectorMuf&, VectorUtf&, VectorMuf&)> calcMuBar,
                      std::function <void(VectorMuf&, MatrixAGf&, MatrixCHf&)> getAandCorGandH
                      );

  void updateStep(VectorMuf& mu_bar, MatrixSigf& sig_bar, MatrixCHf C_or_H, 
                  VectorZtf& zt, MatrixQf& Q,
                  VectorMuf& mu_hat, MatrixSigf& sig_hat,
                  std::function <void(VectorMuf&, VectorZtf&)> calcMeasPred);

};

#endif
