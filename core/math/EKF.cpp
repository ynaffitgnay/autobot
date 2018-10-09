#include "EKF.h"


EKF::EKF() {

}

void EKF::runEKF(VectorMuf& mu_hat, MatrixSigf& sig_hat,VectorUtf& ut, VectorZtf& zt,
                 std::function <void(VectorMuf&, VectorUtf&, VectorMuf&)> calcMuBar,
                 std::function <void(VectorMuf&, MatrixAGf&, MatrixCHf&)> getAandCorGandH,
                 std::function <void(VectorMuf&, VectorZtf&)> calcMeasPred,
                 MatrixQf& Q, MatrixRf& R, bool useMeas) {

    // n is size of state Eigen::Vector
    // m is size of control Eigen::Vector
    // k is the size of the measurement Eigen::Vector
    // where R is the covariance of a Gaussian modelling
    // the randomness in the state transition
  // predictionStep(mu_hat,sig_hat,ut,A,B,R,mu_bar,sig_bar);
  // updateStep(mu_bar,sig_bar,C,zt,C*mu_bar,Q,mu_hat,sig_hat);

    // where C is from z = Cx+del_t
    // del_t is gaussian with mean zero and cov Q

  //Tunable? C Q R

  VectorMuf mu_bar;
  MatrixSigf sig_bar;
  MatrixAGf A_or_G;
  MatrixCHf C_or_H;
  
  if (useMeas) {
    // printf("Ball seen\n");
    predictionStep(mu_hat, sig_hat, ut, A_or_G, C_or_H, R, mu_bar, sig_bar, calcMuBar, getAandCorGandH);
    updateStep(mu_bar, sig_bar, C_or_H, zt, Q, mu_hat, sig_hat, calcMeasPred);
  } else {
    // printf("Ball not seen\n");
    predictionStep(mu_hat, sig_hat, ut, A_or_G, C_or_H, R, mu_bar, sig_bar, calcMuBar, getAandCorGandH);
  }
}


// void EKF::runKF(VectorMuf& mu_hat, MatrixSigf& sig_hat,VectorUtf& ut,
//                 VectorZtf& zt, MatrixAGf& A, MatrixBf& B, MatrixCHf C,
//                 MatrixQf& Q, MatrixRf& R, bool useMeas) {

//     // n is size of state Eigen::Vector
//     // m is size of control Eigen::Vector
//     // k is the size of the measurement Eigen::Vector
//     // where R is the covariance of a Gaussian modelling
//     // the randomness in the state transition
//   // predictionStep(mu_hat,sig_hat,ut,A,B,R,mu_bar,sig_bar);
//   // updateStep(mu_bar,sig_bar,C,zt,C*mu_bar,Q,mu_hat,sig_hat);

//     // where C is from z = Cx+del_t
//     // del_t is gaussian with mean zero and cov Q

//   //Tunable? C Q R

//   if(useMeas){
//     VectorMuf mu_bar;
//     MatrixSigf sig_bar;
//     predictionStep(mu_hat, sig_hat, ut, A, B, R, mu_bar, sig_bar);
//     VectorZtf z_bar = C*mu_bar;
//     updateStep(mu_bar, sig_bar, C, zt, z_bar, Q, mu_hat, sig_hat);
//     // printf("X hat = %.3f\t, Vx hat = %.3f\t, Y hat = %.3f\t, Vy hat = %.3f\n",mu_hat(0),mu_hat(1),mu_hat(2),mu_hat(3));
//     // printf("Px = %.3f\t, Pvx = %.3f\t, Py = %.3f\t, Pvy = %.3f\n",sig_hat(0,0),sig_hat(1,1),sig_hat(2,2),sig_hat(3,3));

//   }
//   else{
//     predictionStep(mu_hat, sig_hat, ut, A, B, R, mu_hat, sig_hat);
//     // printf("X hat = %.3f\t, Vx hat = %.3f\t, Y hat = %.3f\t, Vy hat = %.3f\n",mu_hat(0),mu_hat(1),mu_hat(2),mu_hat(3));
//     // printf("Px = %.3f\t, Pvx = %.3f\t, Py = %.3f\t, Pvy = %.3f\n",sig_hat(0,0),sig_hat(1,1),sig_hat(2,2),sig_hat(3,3));

//   }



// }


void EKF::predictionStep(VectorMuf& mu_hat,MatrixSigf& sig_hat, VectorUtf& ut,
                         MatrixAGf& A_or_G, MatrixCHf& C_or_H, MatrixRf& R,
                         VectorMuf& mu_bar, MatrixSigf& sig_bar,
                         std::function <void(VectorMuf&, VectorUtf&, VectorMuf&)> calcMuBar,
                         std::function <void(VectorMuf&, MatrixAGf&, MatrixCHf&)> getAandCorGandH
                         ) {
  
  // printf("muHat: %f, %f, %f, %f\n", mu_hat(0), mu_hat(1),mu_hat(2),mu_hat(3));
  // printf("PHat: %f, %f, %f, %f\n", sig_hat(0,0), sig_hat(1,1),sig_hat(2,2),sig_hat(3,3));
  calcMuBar(mu_hat, ut, mu_bar);
  getAandCorGandH(mu_bar, A_or_G, C_or_H);
  sig_bar = A_or_G * sig_hat * A_or_G.transpose() + R;
}

void EKF::updateStep(VectorMuf& mu_bar, MatrixSigf& sig_bar, MatrixCHf C_or_H, 
                     VectorZtf& zt, MatrixQf& Q,
                     VectorMuf& mu_hat, MatrixSigf& sig_hat,
                     std::function <void(VectorMuf&, VectorZtf&)> calcMeasPred
                     ) {
  MatrixKf K = sig_bar*(C_or_H.transpose())*(C_or_H*sig_bar*C_or_H.transpose() + Q).inverse();
  VectorZtf z_bar;
  calcMeasPred(mu_bar,z_bar); // for non-linear z_bar is h(mu_bar) for linear it's C*mu_bar
  mu_hat = mu_bar + K*(zt-z_bar); 
  MatrixSigf IMatrix = (K*C_or_H).Identity();
  sig_hat = (IMatrix - K*C_or_H)*sig_bar*(IMatrix - K*C_or_H).transpose()+K*Q*K.transpose();
}

