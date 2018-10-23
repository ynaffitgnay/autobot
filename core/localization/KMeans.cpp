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
  std::vector<Cluster> clusters;

  int numObs = observations.size();
  float minVariance = FLT_MAX;

  // Initialize clusters using Forgy method (randomly choose k points from vector)
  for (int i = 0; i < k_; ++i)
  {
    int randIdx = rand() % numObs;
    
    std::set<int>::const_iterator it = indices.find(randIdx);
    while (it != indices.end()) {
      randIdx = rand() % numObs;
      it = indices.find(randIdx);
    }

    indices.insert(randIdx);
  }

  for (const int& index : indices) {
    Cluster newCluster;
    newCluster.centroid.x = observations.at(index).x;
    newCluster.centroid.y = observations.at(index).y;
    newCluster.centroid.t = observations.at(index).t;

    clusters.push_back(newCluster);
  }

  //int count = 0;
  // Now determine the initial cluster each particle should go into
  for (const Particle& particle : observations) {
    float minMeansSquared = FLT_MAX;
    int minCluster = -1;

    for (int i = 0; i < k_; ++i) {
      float meansSquared = 0;

      //std::cout << "particle.x: " << particle.x << " clusters.at(i).centroid.x: " <<
      //  clusters.at(i).centroid.x << " (particle.x - clusters.at(i).centroid.x): " <<
      //  (particle.x - clusters.at(i).centroid.x) << " pow((particle.x - clusters.at(i).centroid.x), 2): " <<
      //  pow((particle.x - clusters.at(i).centroid.x), 2) << std::endl;
      
      meansSquared += pow((particle.x - clusters.at(i).centroid.x), 2);
      meansSquared += pow((particle.y - clusters.at(i).centroid.y), 2);
      meansSquared += pow((particle.t - clusters.at(i).centroid.t), 2);

      if (meansSquared < minMeansSquared) {
        minMeansSquared = meansSquared;
        minCluster = i;
      }

      //std::cout << "meansSquared of cluster " << i << " is " << meansSquared << std::endl;
    }

    //std::cout << "minCluster of particle " << count++ << " is " << minCluster << "\n\n\n" << std::endl;
    
    if (minCluster == -1) {
        // std::cout << "AAAAAAHHH THIS SHOULDN'T HAPPEN (particle not assigned to any cluster)\n";
        exit(1);
    }

    clusters.at(minCluster).particles.push_back(&particle);
  }

  updateClusters(clusters);

  // while not converged: assign particles then updateclusters
  while (reassignParticles(clusters))
  {
    updateClusters(clusters);
  }

  // now choose the vector with the lowest variance with enough particles
  assignVariances(clusters);

  const Cluster* bestCluster;
  for (const Cluster& cluster : clusters) {
    if (cluster.variance < minVariance && cluster.particles.size() > ((float)numObs / thresholdFactor_)) {
      bestCluster = &cluster;
    }
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
  float totalX, totalY, totalT;//, avgX, avgY, avgT;
  for (Cluster& cluster : clusters) {
    numParticles = cluster.particles.size();

    if (!numParticles) {
      // std::cout << "AHHH !! No particles in this cluster??\n\n\n\n\n\n";
      //exit(1);
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
    std::vector<float> mean = {clusters.at(clusterIdx).centroid.x, clusters.at(clusterIdx).centroid.y, clusters.at(clusterIdx).centroid.t};

    // Assign maximum variance to empty clusters
    if (!clusters.at(clusterIdx).particles.size()) {
      // std::cout << "No particles in this cluster!\n";
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
