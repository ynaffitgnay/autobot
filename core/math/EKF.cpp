#include "EKF.h"


EKF::EKF() {

}

void EKF::runFilter(oldMean,oldCov,ut,zt,A,B,C,Q,newMean,newCov) {
  // tempMean = A.*oldMean + B.*ut
  // tempCov = A.*oldCov.*A' + R 
    // where R is the covariance of a Gaussian modelling
    // the randomness in the state transition

  // K = tempCov.*C'.*(C.*tempCov.*C' + Q)'
    // where C is from z = Cx+del_t
    // del_t is gaussian with mean zero and cov Q
  // newMean = tempMean + K.*(zt-C.*teamMean);
  // newCov = (Identity - K.*C).*tempCov;

  //Tunable? C Q R
}