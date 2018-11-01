#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true), kmeans_(new KMeans(cache, tlogger, 2, 10)), M_(1000), alpha_slow_(0.01), alpha_fast_(0.5),robot_localized_(false) {
}

ParticleFilter::~ParticleFilter() {
  delete kmeans_;
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  w_slow_ = 0.0;
  w_fast_ = 0.0;
  particles().resize(M_);
  auto frame = cache_.frame_info->frame_id;
  for(auto& p : particles()) {
    p.x = Random::inst().sampleU(-1000.0,2000.0);
    p.y = Random::inst().sampleU(-1000.0,1000.0);
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
  if(checkResample()){
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
  int num_unreliable = 0;
  for(auto& p : particles()) {
    double bear_weight = 0.0;
    for(std::map<WorldObjectType,Pose2D>::iterator it=beacons_.begin(); it!=beacons_.end(); ++it){
      auto& beacon_current = cache_.world_object->objects_[it->first];
      if (beacon_current.seen) {
        // printf("Saw %s at [%f, %f] with distance: %f and bearing: %f\n",getName(it->first),it->second.translation.x, it->second.translation.y,beacon_current.visionDistance,beacon_current.visionBearing*180.0/M_PI);
        beacons_list_.insert(it->first);
        double part_dist = sqrt(pow(p.x - it->second.translation.x, 2) + pow(p.y - it->second.translation.y,2));
        double mean_dist = beacon_current.visionDistance;
        double var_dist = (mean_dist/5.0)*(mean_dist/5.0);
        double dist_weight = foldedNormPDF(part_dist,mean_dist,var_dist);

        double part_global_bearing = p.t;  //alpha

        double part_beacon_sep = atan2f(it->second.translation.y-p.y,it->second.translation.x-p.x);  // beta
        double mean_bear = beacon_current.visionBearing;  //theta
        double x_bear = part_beacon_sep - part_global_bearing;  //phi

        double var_bear = 0.2 * 0.2;
        if (x_bear > M_PI  || x_bear < -M_PI) {
          p.t = -p.t;
          x_bear = part_beacon_sep - p.t;
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
  double w_avg = 0;
  double weights_sum = 0;
  double sum_weights_squared = 0.0;
  for(auto& p : particles()){
    weights_sum += p.w;
    w_avg += (p.w / (double)M_);
  }

  w_slow_ += alpha_slow_ * (w_avg - w_slow_);
  w_fast_ += alpha_fast_ * (w_avg - w_fast_);


  if(weights_sum == 0.0) return true;
  for(auto& p : particles()) {
    p.w /= weights_sum;
    sum_weights_squared += p.w*p.w;
  }
  if(sum_weights_squared < 0.000001) return true;
  double N_eff = 1/sum_weights_squared;

  return (N_eff < (M_/4.0));
}

std::vector<Particle> ParticleFilter::resampleStep(){
  std::vector<Particle> resampled_particles(M_);
  bool noob = false;
  if (particles().size() == 0)
  {
    //std::cout << "No particles to resample from!" << std::endl;
    noob = true;
  }

  if (!noob)
  {
    double r = Random::inst().sampleU(0.0, 1.0 / M_);
    double c = particles().at(0).w;
    double u;
    double resample_prob = std::max((1.0 - (w_fast_ / w_slow_)), 0.0);
    int rand_injected = 0;
    int area_injected = 0;
    
    int i = 0;
    //bool noob = false;
    double stdev_x = 10.0;
    double stdev_y = 10.0;
    double stdev_th = 0.01;
    double mu_x = mean_.translation.x;
    double mu_y = mean_.translation.y;
    double mu_th = mean_.rotation;
    
    double N_eff;
    
    for(int m = 0; m < M_; m++){
      u = r + (double(m) - 1.0) / double(M_);
    
      while(u > c){
        ++i;
        if(i >= M_){
          noob = true;
          break;
        }
        c += particles().at(i).w;
      }
    
      if(noob){
        break;
      }
    
      double rand_prob = Random::inst().sampleU(0.0, 1.0);
      if (rand_prob < resample_prob && rand_injected < (0.05 * (double)M_)) {
        if (area_injected < (0.20 * (double)M_)) {
          ++rand_injected;
          ++area_injected;
          particles().at(i).x = stdev_x*Random::inst().sampleN(0.0,1.0) + mu_x;
          particles().at(i).y = stdev_y*Random::inst().sampleN(0.0,1.0) + mu_y;
          particles().at(i).t = stdev_th*Random::inst().sampleN(0.0,1.0) + mu_th;
        } else{
          ++rand_injected;
          particles().at(i).x = Random::inst().sampleU(-1000.0,2000.0);
          particles().at(i).y = Random::inst().sampleU(-1000.0,1000.0);
          particles().at(i).t = Random::inst().sampleU(-M_PI, M_PI);
        }
      }
      particles().at(i).w = 1.0 / M_;
      resampled_particles.at(m) = particles().at(i);
    }
  }
  
  if(noob){
    resampled_particles.clear();
    resampled_particles.resize(M_);
    for(auto& p : resampled_particles) {
      p.x = Random::inst().sampleU(-1000.0,2000.0);
      p.y = Random::inst().sampleU(-1000.0,1000.0);
      p.t = Random::inst().sampleU(-M_PI, M_PI);  
      p.w = 1.0/M_;
    }
  }
  return resampled_particles;
}

