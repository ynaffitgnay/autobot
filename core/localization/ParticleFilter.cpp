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
    printf("Weight: %f\n",p.w);
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
  // Point2D locationDummy;
  // float orientationDummy;
  // kmeans_->runKMeans(particles(), locationDummy, orientationDummy); 
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
      mean_ /= static_cast<float>(particles().size());
    dirty_ = false;
  }
  return mean_;
}

void ParticleFilter::propagationStep(const Pose2D& disp){
  // Equivalent to line 4 where we get proposed state based on particles and control input
  // We might need to account for noise in the disp
  for(auto& p : particles()) {
    // Get the belief
    printf("Before propagation:\n\tParticles Size: %d p.w: %f p.x: %f p.y: %f p.t: %f\n",particles().size(), p.w, p.x,p.y,p.t);
    p.x += disp.translation.x;
    p.y += disp.translation.y;
    p.t += disp.rotation;
    printf("Propposed after propagation:\n\tParticles Size: %d p.w: %f p.x: %f p.y: %f p.t: %f\n",particles().size(), p.w, p.x,p.y,p.t);
  }
}

void ParticleFilter::updateStep(){
  float weights_sum = 0.0;;
  for(auto& p : particles()) {
    for(std::map<WorldObjectType,Pose2D>::iterator it=beacons_.begin(); it!=beacons_.end(); ++it){
      auto& beacon_current = cache_.world_object->objects_[it->first];
      printf("\nBeacon: %s\n", getName(it->first));
      printf("Before importance weighting:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
      p.w *= exp(-pow(sqrt(pow(p.x - it->second.translation.x, 2) + pow(p.y - it->second.translation.y,2)) - beacon_current.visionDistance,2)/(2 * 100.0))/sqrt(2*M_PI*100.0*100.0);
      // TODO: Need to check the sign and range of global orientation and the visionBearing so that they can be added
      p.w *= exp(-pow(atan2f(it->second.translation.x-p.y,it->second.translation.y-p.x) - beacon_current.visionBearing,2)/(2 * 0.2))/sqrt(2*M_PI*0.2*0.2);
    }
    weights_sum += p.w;
  }
  for(auto& p : particles()){
    p.w /= weights_sum;
    printf("After importance weighting:\n\tp.w: %f, p.x: %f p.y: %f p.t: %f\n",p.w,p.x,p.y,p.t);
  }
}

bool ParticleFilter::checkResample(){
  float sum_weights_squared = 0.0;
  for(auto& p : particles()) {
    sum_weights_squared += p.w*p.w;
  }
  float N_eff = 1/sum_weights_squared;
  return N_eff < particles().size()/2.0;
}

std::vector<Particle> ParticleFilter::resampleStep(){
  int M = particles().size();
  std::vector<Particle> resampled_particles;
  float r = Random::inst().sampleU(0.0, 1.0)/M;
  float c = particles()[0].w;
  float u;
  int i = 0;
  for(int m = 0; m < M; m++){
    u = r + (m-1)/M;
    while(u > c){
      i++;
      c += particles()[i].w;
    }
    particles().at(i).w = 1/M;
    resampled_particles.push_back(particles().at(i));
  }
  return resampled_particles;
}
