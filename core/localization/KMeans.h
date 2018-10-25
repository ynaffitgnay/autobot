#ifndef KMEANS_H
#define KMEANS_H

#include <vector>
#include <math/Geometry.h>
#include <math/Pose2D.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>
#include <localization/Particle.h>
#include <memory/WorldObjectBlock.h>

struct Cluster {
  std::vector<const Particle*> particles;
  Particle centroid;
  float variance;
};
  
class KMeans {
  public:
    KMeans(MemoryCache& cache, TextLogger*& tlogger, int k, float threshold);
    int k_;
    float thresholdFactor_;
    Pose2D runKMeans(const std::vector<Particle>& observations);

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    bool reassignParticles(std::vector<Cluster>& clusters);
    void updateClusters(std::vector<Cluster>& clusters);
    void assignVariances(std::vector<Cluster>& clusters);
};
  
#endif
