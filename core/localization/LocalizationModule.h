#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <Eigen/Dense>
#include "math/EKF.h"
#include "math/MatrixDefinitions.h"

class ParticleFilter;
class Point2D;
struct BallLoc {
    MatrixQf Q;
    MatrixAGf A;
    MatrixBf B;
    MatrixCHf C;
    MatrixRf R;
    VectorMuf mu_hat;
    MatrixSigf sig_hat;
};

class LocalizationModule : public Module {
  public:
    BallLoc ball_loc_;
    double last_time_;
    LocalizationModule();
    ~LocalizationModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void initFromWorld();
    void reInit();
    void processFrame();

    void loadParams(LocalizationParams params);
    
    void moveBall(const Point2D& position);
    void movePlayer(const Point2D& position, float orientation);
    void calculateMuBar(VectorMuf& mu_hat, VectorUtf& ut, VectorMuf& mu_bar);
    void calculateGandH(VectorMuf& mu_bar, MatrixAGf& A_or_G, MatrixCHf& C_or_H);
    void calculateMeasPred(VectorMuf& mu_bar, VectorZtf& z_bar);


  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
    ParticleFilter* pfilter_;
    EKF* ekfilter_;

    Eigen::Vector2f rangeToPos(float bearing, float distance);
    Eigen::Vector2f posToRange(float x, float y);

};


