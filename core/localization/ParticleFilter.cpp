#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true), kmeans_(new KMeans(cache, tlogger, 4)), M_(200), alpha_slow_(0.05), alpha_fast_(0.9) {
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

void ParticleFilter::reset(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
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
  
  if (disp.translation.x || disp.translation.y || disp.rotation) {
    propagationStep(disp);
  }
  updateStep();
  printf("\n\n");

  // // Check if resample
  particles() = resampleStep();

  // // Call k means on particles

  // Point2D locationDummy;
  // float orientationDummy;
  // kmeans_->runKMeans(particles(), locationDummy, orientationDummy); 
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // // Compute the mean pose estimate
    // double x_sum;
    // double y_sum;
    // double th_sum;
    // double x_avg;
    // double y_avg;
    // double th_avg;
    // // printf("Getting mean\n");
    // using T = decltype(mean_.translation);
    // for(const auto& p : particles()) {
    //   // printf("\tAdding particle at [%f,%f,%f] with weight: %f\n", p.x,p.y,p.t,p.w);
    //   x_sum += p.x;
    //   y_sum += p.y;
    //   th_sum += p.t;
    // }
    // if(particles().size() > 0) {
    //   x_avg = x_sum/static_cast<double>(particles().size());
    //   y_avg = y_sum/static_cast<double>(particles().size());
    //   th_avg = th_sum/static_cast<double>(particles().size());
    // }
    // mean_.translation = T(x_avg,y_avg);
    // mean_.rotation = th_avg;
    // dirty_ = false;
    mean_ = kmeans_->runKMeans(particles()); 
  }
  return mean_;
}

void ParticleFilter::propagationStep(const Pose2D& disp){
  // Equivalent to line 4 where we get proposed state based on particles and control input
  // We might need to account for noise in the disp
  for(auto& p : particles()) {
    // Get the belief
    float stdevX = 20.0;
    float stdevY = 20.0;
    float stdevTh = 0.05;
    float x_shift = Random::inst().sampleN(disp.translation.x, stdevX);
    float y_shift = Random::inst().sampleN(disp.translation.y, stdevY);
    float theta_shift = Random::inst().sampleN(disp.rotation, stdevTh);
    // printf("Disp mean: [%f,%f,%f] Disp dist result: [%f,%f,%f]\n",disp.translation.x,disp.translation.y,disp.rotation,x_shift,y_shift,theta_shift);
    // printf("Before propagation:\n\tParticles Size: %d p.w: %f p.x: %f p.y: %f p.t: %f\n",particles().size(), p.w, p.x,p.y,p.t);
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
        double part_dist = sqrt(pow(p.x - it->second.translation.x, 2) + pow(p.y - it->second.translation.y,2));
        // printf("\nBeacon: %s Particle Distance: %f Vision Distance: %f\n", getName(it->first),part_dist,beacon_current.visionDistance);
        // printf("X squared: %f Y squared: %f\n", pow(p.x-it->second.translation.x,2),pow(p.y-it->second.translation.y,2));
        // printf("X part: %f Beacon x: %f Y part: %f, Beacon y: %f\n",p.x,it->second.translation.x, p.y, it->second.translation.y);
        // printf("Before importance weighting:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
        double mean_dist = beacon_current.visionDistance;
        double var_dist = (mean_dist/10.0)*(mean_dist/10.0);
        double dist_weight = exp(-pow(part_dist-mean_dist,2)/(2 * var_dist))/sqrt(2*M_PI*var_dist);
        // printf("After importance calc 1:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
        // TODO: Need to check the sign and range of global orientation and the visionBearing so that they can be added
        double part_global_bearing = p.t;  //alpha
        // printf("Part Global Bearing: %f p.t: %f\n", part_global_bearing,p.t);
        double part_beacon_sep = atan2f(it->second.translation.y-p.y,it->second.translation.x-p.x);  // beta
        // double beacon_bear = atan2f(it->second.translation.y,it->second.translation.x);  // beta        
        double mean_bear = beacon_current.visionBearing;  //theta
        double x_bear = part_beacon_sep - part_global_bearing;  //phi
        // printf("Beta: %f Alpha: %f\n", part_beacon_sep,part_global_bearing);
        double var_bear = 0.1*0.1;
        // double bear_weight = (exp(-pow(x_bear-mean_bear,2)/(2 * var_bear))/sqrt(2*M_PI*var_bear));
        // printf("Particle Bearing: %f Vision Bearing: %f\n",x_bear,mean_bear);
        // printf("After importance calc 2:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
        double bear_weight;
        if (x_bear > M_PI/2.0 || x_bear < -M_PI/2.0) {
          p.t = -p.t;
          bear_weight = (exp(-pow(x_bear-mean_bear,2)/(2 * var_bear))/sqrt(2*M_PI*var_bear))*0.1;
        } else {
          bear_weight = (exp(-pow(x_bear-mean_bear,2)/(2 * var_bear))/sqrt(2*M_PI*var_bear));
        }
        p.w *= dist_weight * (0.1 + bear_weight);

      }
    }
    weights_sum += p.w;
    // printf("Sum: %f\n", weights_sum);
  }
  double w_avg = 0;
  for(auto& p : particles()){
    p.w /= weights_sum;
    w_avg += p.w/(double)M_;
    // printf("After re-weighting:\n\tWeight sum: %f p.w: %f, p.x: %f p.y: %f p.t: %f\n",weights_sum, p.w,p.x,p.y,p.t);
  }

  w_slow_ += alpha_slow_*(w_avg - w_slow_);
  w_fast_ += alpha_fast_*(w_avg - w_fast_);

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
  double r = Random::inst().sampleU(0.0, 1.0/M_);
  double c = particles()[0].w;
  double u;
  double resample_prob = (1.0 - (w_fast_ / w_slow_ ) > 0.0) ? 1.0 - (w_fast_ / w_slow_ ) : 0;
  printf("w_fast_: %f w_slow_: %f quotient: %f\n", w_fast_,w_slow_,w_fast_/w_slow_);
  int i = 0;
  for(int m = 0; m < M_; m++){
    u = r + (double(m)-1.0)/double(M_);
    // printf("U: %f R: %f m: %d i: %d c: %f\n",u,r,m,i,c);
    while(u > c){
      i++;
      c += particles()[i].w;
    }
    if(m > (1-resample_prob)*M_ - 1) {
      printf("m: %d resample_prob: %f cast if statement value: %d\n", m, resample_prob,(int)(resample_prob*M_) - 1);
      particles().at(i).x = Random::inst().sampleU(-2500.0,2500.0);
      particles().at(i).y = Random::inst().sampleU(-1250.0,1250.0);
      particles().at(i).t = Random::inst().sampleU(0.0, 2*M_PI);
    }
    particles().at(i).w = 1.0/M_;
    resampled_particles.push_back(particles().at(i));
  }
  return resampled_particles;
}
