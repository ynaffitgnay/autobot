#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true), kmeans_(new KMeans(cache, tlogger, 8, 10)), M_(300), alpha_slow_(0.01), alpha_fast_(0.5),robot_localized_(false) {
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
    p.t = Random::inst().sampleU(-M_PI, M_PI);  
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
  if(checkResample()){
    particles() = resampleStep();
  }
  updateLocalized();
  // printf("Localized: %d",robot_localized_);
  // printf("\n\n");
}

void ParticleFilter::updateLocalized() {
  float fun_threshold = 0.50;
  // printf("w_fast: %f w_slow: %f ratio: %f\n", w_fast_,w_slow_,w_fast_/w_slow_);
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
    if (std::abs(disp.rotation) < 0.05) {
      stdevTh = 0.02;
    }
    if (std::abs(disp.translation.x) < 1.0) {
      stdevX = 10.0;
    }
    if (std::abs(disp.translation.y) < 1.0) {
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
  int num_unreliable = 0;
  for(auto& p : particles()) {
    double bear_weight = 0.0;
    for(std::map<WorldObjectType,Pose2D>::iterator it=beacons_.begin(); it!=beacons_.end(); ++it){
      auto& beacon_current = cache_.world_object->objects_[it->first];
      if (beacon_current.seen) {
        beacons_list_.insert(it->first);
        double part_dist = sqrt(pow(p.x - it->second.translation.x, 2) + pow(p.y - it->second.translation.y,2));
        double mean_dist = beacon_current.visionDistance;
        double var_dist = (mean_dist/10.0)*(mean_dist/10.0);
        double dist_weight = normDist(part_dist,mean_dist,var_dist);
        // printf("After importance calc 1:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);

        double part_global_bearing = p.t;  //alpha
        // printf("Part Global Bearing: %f p.t: %f\n", part_global_bearing,p.t);
        double part_beacon_sep = atan2f(it->second.translation.y-p.y,it->second.translation.x-p.x);  // beta
        double mean_bear = beacon_current.visionBearing;  //theta
        double x_bear = part_beacon_sep - part_global_bearing;  //phi
        // printf("Beta: %f Alpha: %f\n", part_beacon_sep,part_global_bearing);
        double var_bear = 0.1 * 0.1;
        if (x_bear > M_PI  || x_bear < -M_PI) {
          p.t = -p.t;
          x_bear = part_beacon_sep - p.t;
        }
        bear_weight = normDist(x_bear,mean_bear,var_bear);

        // if (beacon_count > 1) {
        p.w *= dist_weight * bear_weight;
        // } else {
        //   p.w *= dist_weight;
        //   num_unreliable++;
        // }
      }
    }
  }
  // if (num_unreliable/M_ > 0.90) {
  //   printf("Too many (%d) unreliable particles. Probably only seeing one beacon\n", num_unreliable);
  // } else {
  //   printf("Unreliable particles: %d Probably seeing multiple beacons\n", num_unreliable);
  // }

  // printf("w_avg: %f, w_fast_: %f w_slow_: %f quotient: %f\n", w_avg, w_fast_,w_slow_,w_fast_/w_slow_);
}

double ParticleFilter::normDist(double x, double mu, double sig_sq) {
  return (exp(-pow(x - mu,2) / (2 * sig_sq)) / sqrt(2 * M_PI * sig_sq));
}

bool ParticleFilter::checkResample(){
  // W slow and fast calculation
  double w_avg = 0;
  double weights_sum = 0;
  double sum_weights_squared = 0.0;
  for(auto& p : particles()){
    weights_sum += p.w;
    w_avg += (p.w / (double)M_);
    // printf("After re-weighting:\n\tWeight sum: %f p.w: %f, p.x: %f p.y: %f p.t: %f\n",weights_sum, p.w,p.x,p.y,p.t);
  }

  // printf("w_avg: %.10f, w_fast_: %f w_slow_: %f quotient: %f\n", w_avg, w_fast_,w_slow_,w_fast_/w_slow_);
  w_slow_ += alpha_slow_ * (w_avg - w_slow_);
  w_fast_ += alpha_fast_ * (w_avg - w_fast_);


  if(weights_sum == 0.0) return true;
  for(auto& p : particles()) {
    // printf("weights: %f\n",p.w);
    p.w /= weights_sum;
    sum_weights_squared += p.w*p.w;
  }
  if(sum_weights_squared < 0.000001) return true;
  double N_eff = 1/sum_weights_squared;
  // printf("Entered check sample\n");
  return (N_eff < (M_/3.0));
}

std::vector<Particle> ParticleFilter::resampleStep(){
  // printf("entered resample step\n");
  std::vector<Particle> resampled_particles(M_);
  double r = Random::inst().sampleU(0.0, 1.0 / M_);
  double c = particles().at(0).w;
  double u;
  double resample_prob = std::max((1.0 - (w_fast_ / w_slow_)), 0.0);
  int rand_injected = 0;
  int area_injected = 0;
  //printf("w_fast_: %f w_slow_: %f quotient: %f\n", w_fast_,w_slow_,w_fast_/w_slow_);
  int i = 0;
  bool noob = false;
  double stdev_x = 10.0;
  double stdev_y = 10.0;
  double stdev_th = 0.01;
  double mu_x = mean_.translation.x;
  double mu_y = mean_.translation.y;
  double mu_th = mean_.rotation;

  double N_eff;

  for(int m = 0; m < M_; m++){
    u = r + (double(m) - 1.0) / double(M_);
    // printf("U: %f R: %f m: %d i: %d c: %f\n",u,r,m,i,c);
    while(u > c){
      ++i;
      if(i >= M_){
        noob = true;
        break;
      }
      // printf("i = %d ", i);
      c += particles().at(i).w;
    }

    if(noob){
      break;
    }

    // printf("no cast: %f, cast: %f\n\n", (1.0 - resample_prob)*M_, (1.0 - resample_prob)*(float)M_);
    double rand_prob = Random::inst().sampleU(0.0, 1.0);
    if (rand_prob < resample_prob && rand_injected < (0.25 * (double)M_)) {
      if (area_injected < (0.20 * (double)M_)) {
        ++rand_injected;
        ++area_injected;
        particles().at(i).x = stdev_x*Random::inst().sampleN(0.0,1.0) + mu_x;
        particles().at(i).y = stdev_y*Random::inst().sampleN(0.0,1.0) + mu_y;
        particles().at(i).t = stdev_th*Random::inst().sampleN(0.0,1.0) + mu_th;
      } else{
        // printf("m: %d resample_prob: %f random_prob: %f\n", m, resample_prob,rand_prob);
        ++rand_injected;
        particles().at(i).x = Random::inst().sampleU(-2500.0,2500.0);
        particles().at(i).y = Random::inst().sampleU(-1250.0,1250.0);
        particles().at(i).t = Random::inst().sampleU(-M_PI, M_PI);
      }
    }
    particles().at(i).w = 1.0 / M_;
    resampled_particles.at(m) = particles().at(i);
  }
  if(noob){
    resampled_particles.clear();
    resampled_particles.resize(M_);
    for(auto& p : resampled_particles) {
      p.x = Random::inst().sampleU(-2500.0,2500.0);
      p.y = Random::inst().sampleU(-1250.0,1250.0);
      p.t = Random::inst().sampleU(-M_PI, M_PI);  
      p.w = 1.0/M_;
    }
  }
  // printf("Random particles injected: %d\n", rand_injected);
  return resampled_particles;
}

