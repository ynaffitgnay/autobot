#include "EKF.h"


EKF::EKF() {

}

void EKF::runEKF(VectorMuf& mu_hat, MatrixSigf& sig_hat,VectorUtf& ut, VectorZtf& zt,
                 std::function <void(VectorMuf&, VectorUtf&, VectorMuf&)> calcMuBar,
                 std::function <void(VectorMuf&, MatrixAGf&, MatrixCHf&)> getAandCorGandH,
                 std::function <void(VectorMuf&, VectorZtf&)> calcMeasPred,
                 MatrixQf& Q, MatrixRf& R, bool useMeas) {

    // R is process noise covariance and Q is the measurement noise covariance matric
  VectorMuf mu_bar;
  MatrixSigf sig_bar;
  MatrixAGf A_or_G;
  MatrixCHf C_or_H;
  
  if (useMeas) {
    // If ball is seen, perform both prediction and update step
    // Prediction increases the covariance of the state while update reduces it making it more accurate
    predictionStep(mu_hat, sig_hat, ut, A_or_G, C_or_H, R, mu_bar, sig_bar, calcMuBar, getAandCorGandH);
    updateStep(mu_bar, sig_bar, C_or_H, zt, Q, mu_hat, sig_hat, calcMeasPred);
  } else {
    // If ball is not seen, only perform the prediction step
    // This increases the covariance
    predictionStep(mu_hat, sig_hat, ut, A_or_G, C_or_H, R, mu_bar, sig_bar, calcMuBar, getAandCorGandH);
  }
}

void EKF::predictionStep(VectorMuf& mu_hat,MatrixSigf& sig_hat, VectorUtf& ut,
                         MatrixAGf& A_or_G, MatrixCHf& C_or_H, MatrixRf& R,
                         VectorMuf& mu_bar, MatrixSigf& sig_bar,
                         std::function <void(VectorMuf&, VectorUtf&, VectorMuf&)> calcMuBar,
                         std::function <void(VectorMuf&, MatrixAGf&, MatrixCHf&)> getAandCorGandH
                         ) {

  // This is the state propagation passed in as a function
  calcMuBar(mu_hat, ut, mu_bar);
  // Get partials of the nonlinear functions g and h w.r.t. the state
  getAandCorGandH(mu_bar, A_or_G, C_or_H);
  // Covariance propagation: Note that this always increases the covariance
  sig_bar = A_or_G * sig_hat * A_or_G.transpose() + R;
}

void EKF::updateStep(VectorMuf& mu_bar, MatrixSigf& sig_bar, MatrixCHf C_or_H, 
                     VectorZtf& zt, MatrixQf& Q,
                     VectorMuf& mu_hat, MatrixSigf& sig_hat,
                     std::function <void(VectorMuf&, VectorZtf&)> calcMeasPred
                     ) {
  // The Kalman gain matrix
  MatrixKf K = sig_bar*(C_or_H.transpose())*(C_or_H*sig_bar*C_or_H.transpose() + Q).inverse();
  VectorZtf z_bar;
  // Calculates the predicted measurement using the propagated state
  calcMeasPred(mu_bar,z_bar);
  // Kalman filter update
  mu_hat = mu_bar + K*(zt-z_bar);

  MatrixSigf IMatrix = (K*C_or_H).Identity();
  // Covariance Update
  sig_hat = (IMatrix - K*C_or_H)*sig_bar*(IMatrix - K*C_or_H).transpose()+K*Q*K.transpose();
}

