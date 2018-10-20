#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true), kmeans_(new KMeans(cache, tlogger, 4)) {
}

ParticleFilter::~ParticleFilter() {
  delete kmeans_;
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;

  int M = 100;  // Number of particles
  particles().resize(M);
  auto frame = cache_.frame_info->frame_id;
  for(auto& p : particles()) {
    p.x = Random::inst().sampleU(-2500.0,2500.0);
    p.y = Random::inst().sampleU(-1250.0,1250.0);
    p.t = Random::inst().sampleU(0.0, 2*M_PI);  
    p.w = 1.0/M;
  }
}

void ParticleFilter::reset(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  int M = 100;  // Number of particles
  auto frame = cache_.frame_info->frame_id;
  for(auto& p : particles()) {
    p.x = Random::inst().sampleU(-2500.0,2500.0);
    p.y = Random::inst().sampleU(-1250.0,1250.0);
    p.t = Random::inst().sampleU(0.0, 2*M_PI);  
    p.w = 1.0/M;
  }
}


void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  // printf("X = %f\t, Y = %f\t, theta = %f\n",disp.translation.x,disp.translation.y,disp.rotation);
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  

  propagationStep(disp);
  updateStep();
  printf("\n\n");

  // // Check if resample
  particles() = resampleStep();

  // // Call k means on particles

  Point2D locationDummy;
  float orientationDummy;
  kmeans_->runKMeans(particles(), locationDummy, orientationDummy); 
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= static_cast<double>(particles().size());
    dirty_ = false;
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
    float stdevTh = 0.2;
    float x_shift = Random::inst().sampleN(disp.translation.x, stdevX);
    float y_shift = Random::inst().sampleN(disp.translation.y, stdevY);
    float theta_shift = Random::inst().sampleN(disp.rotation, stdevTh);
    printf("Disp mean: [%f,%f,%f] Disp dist result: [%f,%f,%f]\n",disp.translation.x,disp.translation.y,disp.rotation,x_shift,y_shift,theta_shift);
    printf("Before propagation:\n\tParticles Size: %d p.w: %f p.x: %f p.y: %f p.t: %f\n",particles().size(), p.w, p.x,p.y,p.t);
    p.x += x_shift;
    p.y += y_shift;
    p.t += theta_shift;
    printf("Propposed after propagation:\n\tParticles Size: %d p.w: %f p.x: %f p.y: %f p.t: %f\n",particles().size(), p.w, p.x,p.y,p.t);

  }
}

void ParticleFilter::updateStep(){
  double weights_sum = 0.0;
  for(auto& p : particles()) {
    for(std::map<WorldObjectType,Pose2D>::iterator it=beacons_.begin(); it!=beacons_.end(); ++it){
      auto& beacon_current = cache_.world_object->objects_[it->first];
      if (beacon_current.seen) {
        double part_dist = sqrt(pow(p.x - it->second.translation.x, 2) + pow(p.y - it->second.translation.y,2));
        printf("\nBeacon: %s Particle Distance: %f Vision Distance: %f\n", getName(it->first),part_dist,beacon_current.visionDistance);
        printf("Before importance weighting:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
        double mean_dist = beacon_current.visionDistance;
        double var_dist = (mean_dist/10.0)*(mean_dist/10.0);
        p.w *= exp(-pow(part_dist-mean_dist,2)/(2 * var_dist))/sqrt(2*M_PI*var_dist);
        printf("After importance calc 1:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
        // TODO: Need to check the sign and range of global orientation and the visionBearing so that they can be added
        double part_bear = atan2f(it->second.translation.x-p.y,it->second.translation.y-p.x);
        double mean_bear = beacon_current.visionBearing;
        double var_bear = 0.3*0.3;
        p.w *= exp(-pow(part_bear-mean_bear,2)/(2 * var_bear))/sqrt(2*M_PI*var_bear);
        printf("Particle Bearing: %f Vision Bearing: %f\n", getName(it->first),part_bear,mean_bear);
        printf("After importance calc 2:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
      }
    }
    weights_sum += p.w;
    printf("Sum: %f\n", weights_sum);
  }
  for(auto& p : particles()){
    p.w /= weights_sum;
    printf("After re-weighting:\n\tWeight sum: %f p.w: %f, p.x: %f p.y: %f p.t: %f\n",weights_sum, p.w,p.x,p.y,p.t);
  }
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
  int M = particles().size();
  std::vector<Particle> resampled_particles;
  double r = Random::inst().sampleU(0.0, 1.0/M);
  double c = particles()[0].w;
  double u;
  int i = 0;
  for(int m = 0; m < M; m++){
    u = r + (double(m)-1.0)/double(M);
    printf("U: %f R: %f m: %d i: %d c: %f\n",u,r,m,i,c);
    while(u > c){
      i++;
      c += particles()[i].w;
    }
    particles().at(i).w = 1.0/M;
    resampled_particles.push_back(particles().at(i));
  }
  return resampled_particles;
}
