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
    std::map<WorldObjectType,Pose2D> landmarks_ = {
    { WO_BEACON_YELLOW_BLUE, Pose2D(0.0, -1530.0,-1280.0) },
    { WO_BEACON_BLUE_YELLOW, Pose2D(0.0, -1530.0,1220.0) },
    { WO_BEACON_YELLOW_PINK, Pose2D(0.0, 1500.0,-1240.0) },
    { WO_BEACON_PINK_YELLOW, Pose2D(0.0, 1500.0,1240.0) },
    { WO_BEACON_BLUE_PINK, Pose2D(0.0, 0.0, 1230.0) },
    { WO_BEACON_PINK_BLUE, Pose2D(0.0, 0.0, -1300.0) },
    { WO_OPP_PEN_LEFT_L, Pose2D(0.0, 900.0, 680.0)},
    { WO_OPP_PEN_RIGHT_L, Pose2D(0.0, 900.0, -690.0)},
    { WO_OPP_PEN_LEFT_T, Pose2D(0.0, 1500.0, 700.0)},
    { WO_OPP_PEN_RIGHT_T, Pose2D(0.0, 1500.0, -650.0)},
    { WO_OWN_PEN_RIGHT_T, Pose2D(0.0, -1530.0, 650.0)},
    { WO_OWN_PEN_LEFT_T, Pose2D(0.0, -1500.0, -700.0)}};
    // { WO_OBSTACLE_1, Pose2D(0.0, 1800.0, 750.0) },
    // { WO_OBSTACLE_2, Pose2D(0.0, 750.0, 1350.0) },

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    KMeans* kmeans_;
    int M_;
    bool robot_localized_;
    mutable Pose2D mean_;
    mutable bool dirty_;
    void propagationStep(const Pose2D& disp);
    void updateStep();
    bool checkResample();
    std::vector<Particle> resampleStep();
    double normPDF(double x, double mu, double sig_sq);
    double foldedNormPDF(double x, double mu, double sig_sq);
    double truncNormPDF(double x, double mu, double sig_sq, double a, double b);
    bool noMeasurement();
};

#endif
