#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <Eigen/Dense>
#include "math/EKF.h"

class ParticleFilter;
class Point2D;
struct BallLoc {
    Eigen::Matrix2f Q;
    Eigen::Matrix4f A;
    Eigen::Matrix4f B;
    Matrix24f C;
    Eigen::Matrix4f R;
    Eigen::Vector4f mu_hat;
    Eigen::Matrix4f sig_hat;
    Eigen::Vector4f mu_bar;
    Eigen::Matrix4f sig_bar;
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
  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
    ParticleFilter* pfilter_;
    EKF* ekfilter_;

    Eigen::Vector2f rangeToPos(float bearing, float distance);
    Eigen::Vector2f posToRange(float x, float y);

};


