#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true), kmeans_(new KMeans(cache, tlogger, 3, 10)), M_(1000), robot_localized_(false) {
}

ParticleFilter::~ParticleFilter() {
  delete kmeans_;
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  particles().resize(M_);
  auto frame = cache_.frame_info->frame_id;
  for(auto& p : particles()) {
    p.x = Random::inst().sampleN(1250.0,10.0);
    p.y = Random::inst().sampleN(-1000.0,10.0);
    p.t = Random::inst().sampleU(-M_PI, M_PI);
    p.w = 1.0/M_;
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;

  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  propagationStep(disp);
  
  updateStep();
  
  // Check if resample
  if(checkResample() && !noMeasurement()){
    particles() = resampleStep();
  }
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    Pose2D newPose = kmeans_->runKMeans(particles());
    if (!(newPose.translation.x == -15000.0 &&
        newPose.translation.y == -15000.0 &&
          newPose.rotation == -15000.0))
    {
      mean_ = newPose;
    }
  }
  return mean_;
}

void ParticleFilter::propagationStep(const Pose2D& disp) {
  // Equivalent to line 4 where we get proposed state based on particles and control input
  for(auto& p : particles()) {
    // Get the belief
    float stdevX = 10.0;
    float stdevY = 10.0;
    float stdevTh = 0.02; 
  
    if (std::abs(disp.rotation) < 0.01) {
      stdevTh = 0.005;
    }
    if (std::abs(disp.translation.x) < 0.75) {
      stdevX = 5.0;
    }
    if (std::abs(disp.translation.y) < 0.75) {
      stdevY = 5.0;
    }

    float zero = 0.0;
    float x_shift = Random::inst().sampleN(zero, stdevX);
    float y_shift = Random::inst().sampleN(zero, stdevY);
    float theta_shift = Random::inst().sampleN(zero, stdevTh);
  
    p.x += (x_shift+disp.translation.x)*cos(p.t)+(y_shift+disp.translation.y)*(cos(p.t+(M_PI/2.0)));
    p.y += (x_shift+disp.translation.x)*sin(p.t)+(y_shift+disp.translation.y)*(sin(p.t+(M_PI/2.0)));
    p.t += theta_shift+disp.rotation;
  
  }
}

void ParticleFilter::updateStep(){
  for(auto& p : particles()) {
    double bear_weight = 0.0;
    for(std::map<WorldObjectType,Pose2D>::iterator it=landmarks_.begin(); it!=landmarks_.end(); ++it){
      auto& landmark_current = cache_.world_object->objects_[it->first];
      if (landmark_current.seen) {
        double part_dist = sqrt(pow(p.x - it->second.translation.x, 2) + pow(p.y - it->second.translation.y,2));
        double mean_dist = landmark_current.visionDistance;
        double var_dist = (mean_dist/3.5)*(mean_dist/3.5);
        double dist_weight = foldedNormPDF(part_dist,mean_dist,var_dist);

        double part_global_bearing = p.t;

        double part_landmark_sep = atan2f(it->second.translation.y-p.y,it->second.translation.x-p.x);  // beta
        double mean_bear = landmark_current.visionBearing;  //theta
        double x_bear = part_landmark_sep - part_global_bearing;  //phi

        double var_bear = 0.3 * 0.3;
        if (x_bear > M_PI  || x_bear < -M_PI) {
          p.t = -p.t;
          x_bear = part_landmark_sep - p.t;
        }
        bear_weight = truncNormPDF(x_bear,mean_bear,var_bear,-M_PI,M_PI);

        p.w *= dist_weight * bear_weight;
      }
    }
  }
}

double ParticleFilter::normPDF(double x, double mu, double sig_sq) {
  return (exp(-pow(x - mu,2) / (2 * sig_sq)) / sqrt(2 * M_PI * sig_sq));
}

double ParticleFilter::foldedNormPDF(double x, double mu, double sig_sq) {
  return normPDF(x,mu,sig_sq) + normPDF(-x,mu,sig_sq);
}

double ParticleFilter::truncNormPDF(double x, double mu, double sig_sq, double a, double b) {
  if( (x < b) && (x > a)){
    return normPDF(x,mu,sig_sq);
  }
  else {
    return 0.0;
  }
}

bool ParticleFilter::checkResample(){
  // W slow and fast calculation
  double weights_sum = 0;
  double sum_weights_squared = 0.0;
  for(auto& p : particles()){
    weights_sum += p.w;
  }

  if(weights_sum == 0.0) return true;
  for(auto& p : particles()) {
    p.w /= weights_sum;
    sum_weights_squared += p.w*p.w;
  }
  if(sum_weights_squared < 0.000001) return true;
  double N_eff = 1/sum_weights_squared;

  return (N_eff < (0.9*M_));
}

bool ParticleFilter::noMeasurement(){
  for(std::map<WorldObjectType,Pose2D>::iterator it=landmarks_.begin(); it!=landmarks_.end(); ++it){
    auto& landmark_current = cache_.world_object->objects_[it->first];
    if (landmark_current.seen) return false;
  }
  return true;
}

std::vector<Particle> ParticleFilter::resampleStep(){
  std::vector<Particle> resampled_particles(M_);
  if (particles().size() == 0){
    double  stdev_x = 15.0,
            stdev_y = 15.0,
            stdev_th = 0.1,
            mu_x = mean_.translation.x,
            mu_y = mean_.translation.y,
            mu_th = mean_.rotation;
    resampled_particles.clear();
    resampled_particles.resize(M_);
    for(auto& p : resampled_particles) {
      p.x = Random::inst().sampleN(mu_x,stdev_x);
      p.y = Random::inst().sampleN(mu_y,stdev_y);
      p.t = Random::inst().sampleN(mu_th,stdev_th);
      p.w = 1.0/M_;
    }
    return resampled_particles;
  }

  double r = Random::inst().sampleU(1.0);
  int k = 0;
  double pcum = particles().at(k).w;
  for(int j = 0; j < M_; j++){
    while((pcum < (j+r)/M_) && (k < M_-1))  pcum += particles().at(++k).w;
    resampled_particles.at(j) = particles().at(k);
  }
  return resampled_particles;
}