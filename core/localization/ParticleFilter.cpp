#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true), kmeans_(new KMeans(cache, tlogger, 8, 10)), M_(200), alpha_slow_(0.01), alpha_fast_(0.8),robot_localized_(false) {
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
    p.x = Random::inst().sampleU(-2500.0,2500.0);
    p.y = Random::inst().sampleU(-1250.0,1250.0);
    p.t = Random::inst().sampleU(0.0, 2*M_PI);  
    p.w = 1.0/M_;
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  // printf("X = %f\t, Y = %f\t, theta = %f\n",disp.translation.x,disp.translation.y,disp.rotation);
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  // printf("Disp mean: [%f,%f,%f]\n",disp.translation.x,disp.translation.y,disp.rotation);
  // if (std::abs(disp.rotation) > 0.05 || std::abs(disp.translation.x) > 1.0 || std::abs(disp.translation.y) > 1.0) {
  propagationStep(disp);
  // }
  updateStep();
  // Check if resample
  particles() = resampleStep();
  updateLocalized();
  printf("Localized: %d",robot_localized_);
  printf("\n\n");

}

void ParticleFilter::updateLocalized() {
  float fun_threshold = 0.50;
  printf("w_fast: %f w_slow: %f ratio: %f\n", w_fast_,w_slow_,w_fast_/w_slow_);
  if (w_fast_/w_slow_ > fun_threshold) {
    // Might need to be some more conditions to prevent jittering
    if (beacons_list_.size() >= 2) {
      robot_localized_ = true;
    }
  } else {
    // Might need to be some more conditions to prevent jittering
    robot_localized_ = false;
    beacons_list_.clear();
  }
}

bool ParticleFilter::getLocalized() {
  return robot_localized_;
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    mean_ = kmeans_->runKMeans(particles()); 
  }
  return mean_;
}

void ParticleFilter::propagationStep(const Pose2D& disp) {
  // Equivalent to line 4 where we get proposed state based on particles and control input
  // printf("In prop step\n");
  for(auto& p : particles()) {
    // Get the belief
    float stdevX = 20.0; 
    float stdevY = 20.0; 
    float stdevTh = 0.05; 
    // printf("Before propagation:\n\tParticles Size: %d p.w: %f p.x: %f p.y: %f p.t: %f\n",particles().size(), p.w, p.x,p.y,p.t);
    if (std::abs(disp.rotation) < 0.01) {
      stdevTh = 0.02;
    }
    if (std::abs(disp.translation.x) < 0.75) {
      stdevX = 10.0;
    }
    if (std::abs(disp.translation.y) < 0.75) {
      stdevY = 10.0;
    }
    float x_shift = Random::inst().sampleN(disp.translation.x, stdevX);
    float y_shift = Random::inst().sampleN(disp.translation.y, stdevY);
    float theta_shift = Random::inst().sampleN(disp.rotation, stdevTh);
    // printf("Disp mean: [%f,%f,%f] Disp dist result: [%f,%f,%f]\n",disp.translation.x,disp.translation.y,disp.rotation,x_shift,y_shift,theta_shift);
    p.x += x_shift+(disp.translation.x)*cos(p.t)+(disp.translation.y)*(cos(p.t+(M_PI/2.0)));
    p.y += y_shift+(disp.translation.x)*sin(p.t)+(disp.translation.y)*(sin(p.t+(M_PI/2.0)));
    p.t += theta_shift+disp.rotation;
    // printf("Proposed after propagation:\n\tParticles Size: %d p.w: %f p.x: %f p.y: %f p.t: %f\n",particles().size(), p.w, p.x,p.y,p.t);
  }
}

void ParticleFilter::updateStep(){
  double weights_sum = 0.0;
  for(auto& p : particles()) {
    for(std::map<WorldObjectType,Pose2D>::iterator it=beacons_.begin(); it!=beacons_.end(); ++it){
      auto& beacon_current = cache_.world_object->objects_[it->first];
      if (beacon_current.seen) {
        beacons_list_.insert(it->first);
        double part_dist = sqrt(pow(p.x - it->second.translation.x, 2) + pow(p.y - it->second.translation.y,2));
        double mean_dist = beacon_current.visionDistance;
        double var_dist = (mean_dist/10.0)*(mean_dist/10.0);
        double dist_weight = exp(-pow(part_dist-mean_dist,2)/(2 * var_dist))/sqrt(2*M_PI*var_dist);
        // printf("After importance calc 1:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);


        double part_global_bearing = p.t;  //alpha
        // printf("Part Global Bearing: %f p.t: %f\n", part_global_bearing,p.t);
        double part_beacon_sep = atan2f(it->second.translation.y-p.y,it->second.translation.x-p.x);  // beta
        double mean_bear = beacon_current.visionBearing;  //theta
        double x_bear = part_beacon_sep - part_global_bearing;  //phi
        // printf("Beta: %f Alpha: %f\n", part_beacon_sep,part_global_bearing);
        double var_bear = 0.15 * 0.15;
        double bear_weight;
        if (x_bear > M_PI  || x_bear < -M_PI) {
          p.t = -p.t;
          bear_weight = (exp(-pow(x_bear - mean_bear,2) / (2 * var_bear)) / sqrt(2 * M_PI * var_bear));//0.1;
        } else {
          bear_weight = (exp(-pow(x_bear - mean_bear,2) / (2 * var_bear)) / sqrt(2 * M_PI * var_bear));
        }


        p.w *= dist_weight * (0.1 + bear_weight);
      } 
    }
    weights_sum += p.w;
    // printf("Sum: %f\n", weights_sum);
  }
  double w_avg = 0;
  for(auto& p : particles()){
    w_avg += (p.w / (double)M_);
    p.w /= weights_sum;
    // printf("After re-weighting:\n\tWeight sum: %f p.w: %f, p.x: %f p.y: %f p.t: %f\n",weights_sum, p.w,p.x,p.y,p.t);
  }

  // printf("w_avg: %.10f, w_fast_: %f w_slow_: %f quotient: %f\n", w_avg, w_fast_,w_slow_,w_fast_/w_slow_);
  w_slow_ += alpha_slow_ * (w_avg - w_slow_);
  w_fast_ += alpha_fast_ * (w_avg - w_fast_);

  // printf("w_avg: %f, w_fast_: %f w_slow_: %f quotient: %f\n", w_avg, w_fast_,w_slow_,w_fast_/w_slow_);
}

bool ParticleFilter::checkResample(){
  double sum_weights_squared = 0.0;
  for(auto& p : particles()) {
    sum_weights_squared += p.w*p.w;
  }
  double N_eff = 1/sum_weights_squared;
  return N_eff < particles().size()/2.0;
}

std::vector<Particle> ParticleFilter::resampleStep(){
  std::vector<Particle> resampled_particles;
  double r = Random::inst().sampleU(0.0, 1.0 / M_);
  double c = particles()[0].w;
  double u;
  double resample_prob = std::max((1.0 - (w_fast_ / w_slow_)), 0.0);
  int rand_injected = 0;
  //printf("w_fast_: %f w_slow_: %f quotient: %f\n", w_fast_,w_slow_,w_fast_/w_slow_);
  int i = 0;
  for(int m = 0; m < M_; m++){
    u = r + (double(m) - 1.0) / double(M_);
    // printf("U: %f R: %f m: %d i: %d c: %f\n",u,r,m,i,c);
    while(u > c){
      i++;
      c += particles()[i].w;
    }
    //printf("no cast: %f, cast: %f\n\n", (1.0 - resample_prob)*M_, (1.0 - resample_prob)*(float)M_);
    double rand_prob = Random::inst().sampleU(0.0, 1.0);
    if (rand_prob < resample_prob && rand_injected < ((double)M_) / 3.0) {
      ++rand_injected;
      // printf("m: %d resample_prob: %f random_prob: %f\n", m, resample_prob,rand_prob);
      particles().at(i).x = Random::inst().sampleU(-2500.0,2500.0);
      particles().at(i).y = Random::inst().sampleU(-1250.0,1250.0);
      particles().at(i).t = Random::inst().sampleU(0.0, 2*M_PI);
    }
    particles().at(i).w = 1.0 / M_;
    resampled_particles.push_back(particles().at(i));
  }
  printf("Random particles injected: %d\n", rand_injected);
  return resampled_particles;
}
