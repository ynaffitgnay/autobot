#include <localization/KMeans.h>
#include <stdlib.h>
#include <set>
#include <float.h>
#include <stdio.h>
#include <cmath>
#include <Eigen/LU>

KMeans::KMeans(MemoryCache& cache, TextLogger*& tlogger, int k, float threshold)
  : cache_(cache), tlogger_(tlogger), k_(k), thresholdFactor_(threshold) {
}

Pose2D KMeans::runKMeans(const std::vector<Particle>& observations) {
  std::set<int> indices;
  std::vector<Cluster> clusters(k_);

  int numObs = observations.size();
  float minVariance = FLT_MAX;

  Pose2D emptyPose = Pose2D();
  emptyPose.translation.x = -15000.0;
  emptyPose.translation.y = -15000.0;
  emptyPose.rotation = -15000.0;
  
  if (numObs == 0)
  {
    std::cout << "No particles to determine pose!" << std::endl;
    return emptyPose;
  }
  
  // Initialize clusters using random partition method
  for (const Particle& particle : observations) {
    int clusterIdx = rand() % k_;
    clusters.at(clusterIdx).particles.push_back(&particle);
  }

  updateClusters(clusters);

  int count = 0;
  
  // while not converged: assign particles then updateclusters
  while (reassignParticles(clusters))
  {
    ++count;
    updateClusters(clusters);
  }

  //std::cout << "Iterations until convergence: " << count << std::endl;

  // now choose the vector with the lowest variance with enough particles
  assignVariances(clusters);

  const Cluster* bestCluster = NULL;
  for (const Cluster& cluster : clusters) {
    if (cluster.variance < minVariance && cluster.particles.size() > ((float)numObs / thresholdFactor_)) {
      minVariance = cluster.variance;
      bestCluster = &cluster;
    }
  }

  if (bestCluster == NULL || minVariance == FLT_MAX)
  {
    std::cout << "No bestCluster\n";
    return emptyPose;
  }
  
  Pose2D clusterLoc = Pose2D();

  clusterLoc.translation.x = bestCluster->centroid.x;
  clusterLoc.translation.y = bestCluster->centroid.y;
  clusterLoc.rotation = bestCluster->centroid.t;

  return clusterLoc;
}

// Return false if no particles reassigned!
bool KMeans::reassignParticles(std::vector<Cluster>& clusters) {
  bool noneReassigned = true;
  std::vector<Cluster>::iterator clusterIt;
  int clusterIdx = 0;
  std::vector<std::vector<const Particle*>> newClusterAssignments(k_);

  if (clusters.size() != k_)
  {
    std::cout << "Not k clusters!" << std::endl;
    return !noneReassigned;
  }

  for (clusterIt = clusters.begin(); clusterIt != clusters.end(); ++clusterIt) {
    for (auto& particlePtr : clusterIt->particles) {
      float minMeansSquared = FLT_MAX;
      int minCluster = -1;
    
      for (int i = 0; i < k_; ++i) {
        float meansSquared = 0;
        
        meansSquared += pow((particlePtr->x - clusters.at(i).centroid.x), 2);
        meansSquared += pow((particlePtr->y - clusters.at(i).centroid.y), 2);
        meansSquared += pow((particlePtr->t - clusters.at(i).centroid.t), 2);
    
        if (meansSquared < minMeansSquared) {
          minMeansSquared = meansSquared;
          minCluster = i;
        }
      }
      
      if (minCluster == -1) {
          // std::cout << "AAAAAAHHH THIS SHOULDN'T HAPPEN\n";
          exit(1);
      }

      // new cluster is different from old cluster
      if (minCluster != clusterIdx) noneReassigned = false;
    
      newClusterAssignments.at(minCluster).push_back(particlePtr);
    }
    ++clusterIdx;
  }
  
  for (int i = 0; i < k_; ++i) {
    clusters.at(i).particles = newClusterAssignments.at(i);
  }
  
  return !noneReassigned;
}

void KMeans::updateClusters(std::vector<Cluster>& clusters) {
  int numParticles;
  float totalX, totalY, totalT;
  for (Cluster& cluster : clusters) {
    numParticles = cluster.particles.size();

    if (!numParticles) {
      continue;
    }
    
    totalX = 0;
    totalY = 0;
    totalT = 0;

    for (const auto& particle : cluster.particles) {
      totalX += particle->x;
      totalY += particle->y;
      totalT += particle->t;
    }

    cluster.centroid.x = totalX / numParticles;
    cluster.centroid.y = totalY / numParticles;
    cluster.centroid.t = totalT / numParticles;
  }
}

void KMeans::assignVariances(std::vector<Cluster>& clusters) {
  // calculate the variance for each cluster
  typedef Eigen::Matrix<float,3,3> Matrix3f;

  for (int clusterIdx = 0; clusterIdx < k_; ++clusterIdx) {
    Matrix3f covarianceMat = Matrix3f::Zero(); // new covariance matrix for each cluster

    if (clusters.size() != k_) {
      std::cout << "Wrong number of clusters!" << std::endl;
      for (auto& cluster : clusters) {
        cluster.variance = FLT_MAX;
      }
      return;  
    }
    
    std::vector<float> mean = {clusters.at(clusterIdx).centroid.x, clusters.at(clusterIdx).centroid.y, clusters.at(clusterIdx).centroid.t};

    // Assign maximum variance to empty clusters
    if (!clusters.at(clusterIdx).particles.size()) {
      clusters.at(clusterIdx).variance = FLT_MAX;
      continue;
    }
    
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::vector<const Particle*>::const_iterator it;
        for (it = clusters.at(clusterIdx).particles.begin(); it != clusters.at(clusterIdx).particles.end(); ++it) {
          std::vector<float> particle = {(*it)->x,(*it)->y,(*it)->t};
          covarianceMat(i,j) += ((particle[i] - mean[i]) * (particle[j] - mean[j]));
        }
        covarianceMat(i,j) /= (clusters.at(clusterIdx).particles.size() - 1);
      }
    }
    // Now calculate the determinant
    clusters.at(clusterIdx).variance = covarianceMat.determinant();
  }
}
