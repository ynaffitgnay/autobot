#include "EKF.h"


EKF::EKF() {

}

void EKF::runFilter(Eigen::Vector4f oldMean,Eigen::Matrix4f oldCov,Eigen::Vector4f ut,
                    Eigen::Vector2f zt, Eigen::Matrix4f A, Eigen::Matrix4f B, Matrix24f C,
                    Eigen::Matrix2f Q, Eigen::Matrix4f R, Eigen::Vector4f& mu_hat, 
                    Eigen::Matrix4f& sig_hat) {

    // n is size of state Eigen::Vector
    // m is size of control Eigen::Vector
    // k is the size of the measurement Eigen::Vector
    // where R is the covariance of a Gaussian modelling
    // the randomness in the state transition
  Eigen::Vector4f mu_bar;
  Eigen::Matrix4f sig_bar;
  predictionStep(oldMean,oldCov,ut,A,B,R,mu_bar,sig_bar);
  updateStep(mu_bar,sig_bar,C,zt,C*mu_bar,Q,mu_hat,sig_hat);

    // where C is from z = Cx+del_t
    // del_t is gaussian with mean zero and cov Q

  //Tunable? C Q R
}

void EKF::predictionStep(Eigen::Vector4f oldMean,Eigen::Matrix4f oldCov,Eigen::Vector4f ut,
                        Eigen::Matrix4f A, Eigen::Matrix4f B, Eigen::Matrix4f R,
                        Eigen::Vector4f& mu_bar, Eigen::Matrix4f& sig_bar) {
  mu_bar = A*oldMean + B*ut;
  sig_bar = A*oldCov*A.transpose() + R ;
}

void EKF::updateStep(Eigen::Vector4f mu_bar, Eigen::Matrix4f sig_bar, Matrix24f C, 
                     Eigen::Vector2f zt, Eigen::Vector2f z_bar, Eigen::Matrix2f Q,
                     Eigen::Vector4f& mu_hat, Eigen::Matrix4f& sig_hat) {
  Matrix42f K = sig_bar*C.transpose()*(C*sig_bar*C.transpose() + Q).transpose();
  mu_hat = mu_bar + K*(zt-C*mu_bar);
  Eigen::Matrix4f IMatrix = (K*C).Identity();
  sig_hat = (IMatrix - K*C)*sig_bar*(IMatrix - K*C).transpose()+K*Q*K.transpose();
}

