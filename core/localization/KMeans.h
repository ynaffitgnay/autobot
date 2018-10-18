#ifndef KMEANS_H
#define KMEANS_H

#include <vector>
#include <math/Geometry.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>
#include <localization/Particle.h>
#include <memory/WorldObjectBlock.h>

struct Cluster {
  //int id;
  std::vector<Particle*> particles;
  float mean;
  float variance;
};
  
class KMeans {
  public:
    KMeans(MemoryCache& cache, TextLogger*& tlogger, int k);
    int k_;
    void runKMeans(const std::vector<Particle>& observations, Point2D& loc, float& orientation);

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    void assignParticles(std::vector<std::vector<Particle*>>& clusters);
    void updateClusters(std::vector<std::vector<Particle*>>& clusters);
    
};
  
#endif
