#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#pragma once

#include <math/Pose2D.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <memory/WorldObjectBlock.h>
#include <localization/Logging.h>
#include <localization/KMeans.h>
#include <common/Particle.h>

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    ~ParticleFilter();
    void init(Point2D loc, float orientation);
    void processFrame();
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }
    std::map<WorldObjectType,Pose2D> beacons_ = {
    { WO_BEACON_YELLOW_BLUE, Pose2D(0.0, -500.0,-1000.0) },
    { WO_BEACON_BLUE_YELLOW, Pose2D(0.0, -500.0,-1000.0) },
    { WO_BEACON_YELLOW_PINK, Pose2D(0.0, 500.0,-1000.0) },
    { WO_BEACON_PINK_YELLOW, Pose2D(0.0, 500.0,-1000.0) },
    { WO_BEACON_BLUE_PINK, Pose2D(0.0, 0.0, 1000.0) },
    { WO_BEACON_PINK_BLUE, Pose2D(0.0, 0.0, 1000.0) }};

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    KMeans* kmeans_;
    int M_;
    double w_slow_;
    double w_fast_;
    double alpha_slow_;
    double alpha_fast_;
    bool robot_localized_;
    std::set<WorldObjectType> beacons_list_;
    mutable Pose2D mean_;
    mutable bool dirty_;
    void propagationStep(const Pose2D& disp);
    void updateStep();
    bool checkResample();
    std::vector<Particle> resampleStep();
    double normDist(double x, double mu, double sig_sq);
};

#endif
