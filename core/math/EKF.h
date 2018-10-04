#ifndef _EKF_H
#define _EKF_H

#include <cmath>
#include <iostream>

/**
 * Extended Kalman Filter Implementation
 */
class EKF
{
public:
  /** 
   * Default constructor. 
   */
  EKF() {}

  /**
   * Constructor.
   */
  EKF();

  void linearizeData(ut,xOld);
  void runFilter(oldMean,oldCov,ut,zt,A,B,C,Q,newMean,newCov);

}

#endif