#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>
#include <math/Geometry.h>

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), ekfilter_(new EKF()), pfilter_(new ParticleFilter(cache_, tlogger_)) {
  // The ball was kept stationary to calculate Q value
  ball_loc_.Q << 0.001,0,
                0,30;

  // This value was 
  ball_loc_.R << 1,0,0,0,
                0,2,0,0,
                0,0,1,0,
                0,0,0,2;

  // The initial state distribution was chosen randomly
  ball_loc_.mu_hat << 50,0,50,0;

  ball_loc_.sig_hat << 10,0,0,0,
                      0,20,0,0,
                      0,0,10,0,
                      0,0,0,20;

  // The state transition matrix is initialized here and updated according to delta_t at all times
  ball_loc_.A << 1,0.33,0,0,
                0,1,0,0,
                0,0,1,0.33,
                0,0,0,1;
  ball_loc_.B << 0,0,0,0,
                0,0,0,0,
                0,0,0,0,
                0,0,0,0;
  // Only for linear: x and y measurement
  ball_loc_.C << 1,0,0,0,
                0,0,1,0;

  // Initialized at negative value and updated every frame
  last_time_ = -1.0;

  // The kalman filter is re-initialized when the ball is not seen for some time
  occluded_time = 0.0;
  pose_index_ = 0;
}

LocalizationModule::~LocalizationModule() {
  delete pfilter_;
  delete ekfilter_;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  printf("initializing particle filter");
  pfilter_->init(self.loc, self.orientation);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  pfilter_->init(Point2D(-750,0), 0.0f);
  cache_.localization_mem->player_ = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

Pose2D LocalizationModule::avgLocVals(Pose2D pose) {
  int window_size = 10;
  double x_sum;
  double y_sum;
  double th_sum;
  if (pose_list_.size() <= window_size) {
    pose_list_.push_back(pose);
  } else {
    pose_list_.at(pose_index_++ % window_size) = pose; 
  }
  for (auto& p : pose_list_) {
      x_sum += p.translation.x;
      y_sum += p.translation.y;
      th_sum += p.rotation;
  }
  Pose2D ret_pose = Pose2D();
  ret_pose.translation.x = x_sum/(double)pose_list_.size();
  ret_pose.translation.y = y_sum/(double)pose_list_.size();
  ret_pose.rotation = th_sum/(double)pose_list_.size();
  return ret_pose;
}

void LocalizationModule::processFrame() {
  double time = cache_.frame_info->seconds_since_start;
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Process the current frame and retrieve our location/orientation estimate
  // from the particle filter
  pfilter_->processFrame(); 
  Pose2D pose_avg; 
  pose_avg = avgLocVals(pfilter_->pose());
  self.loc = pose_avg.translation;
  self.orientation = pose_avg.rotation;

  log(40, "Localization Update: x=%2.f, y=%2.f, theta=%2.2f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);
  double dt = (last_time_ < 0) ? 1.0/30.0 : (time - last_time_);
  if(!ball.seen){
    occluded_time += dt; // Update time for which ball is not seen
  }

  ball_loc_.A << 1,dt,0,0,
                 0,1,0,0,
                 0,0,1,dt,
                 0,0,0,1;
  auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);


  Eigen::Vector2f pos;
  
  // Linear case: x and y measurements
  // pos = rangeToPos(ball.visionBearing,ball.visionDistance); // For linear KF only

  // Nonlinear case: bearing and distance measurements
  pos << ball.visionBearing,ball.visionDistance; // for EKF

  VectorMuf mu_hat = ball_loc_.mu_hat;
  MatrixSigf sig_hat = ball_loc_.sig_hat;
  Eigen::Vector4f ut(0,0,0,0); // There is no control input in this system

  ekfilter_->runEKF(ball_loc_.mu_hat, ball_loc_.sig_hat, ut, pos,
                    std::bind(&LocalizationModule::calculateMuBar, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                    std::bind(&LocalizationModule::calculateGandH, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                    std::bind(&LocalizationModule::calculateMeasPred, this, std::placeholders::_1, std::placeholders::_2),
                    ball_loc_.Q, ball_loc_.R, ball.seen);
  
  if(occluded_time >= 2.0){ // If ball not seen for 2 sec, re-initialize state and covariance
    ball_loc_.mu_hat(0) = relBall.x/10.0;
    ball_loc_.mu_hat(1) = 0.0;
    ball_loc_.mu_hat(2) = relBall.y/10.0;
    ball_loc_.mu_hat(3) = 0.0;
    ball_loc_.sig_hat << 10,0,0,0,
                          0,20,0,0,
                          0,0,10,0,
                          0,0,0,20;
    occluded_time = 0.0;
  }

  ball.relVel.x = ball_loc_.mu_hat(1); // Fill in the x velocity
  ball.relVel.y = ball_loc_.mu_hat(3); // Fill in the y velocity
  auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

  // Compute the global position of the ball based on our assumed position and orientation
  Eigen::Vector2f range = posToRange(ball_loc_.mu_hat(0),ball_loc_.mu_hat(2));

  // Update the ball in the WorldObject block so that it can be accessed in python
  ball.loc = globalBall;
  ball.distance = range(0);
  ball.bearing = range(1);
  cache_.localization_mem->state[0] = ball.loc.x;
  cache_.localization_mem->state[1] = ball.loc.y;
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
  last_time_ = cache_.frame_info->seconds_since_start;
}


Eigen::Vector2f LocalizationModule::rangeToPos(float bearing, float distance) {
  Eigen::Vector2f pos(distance*cos(bearing)/10.0,-distance*sin(bearing)/10.0);
  return pos;
}

Eigen::Vector2f LocalizationModule::posToRange(float x, float y) {
  // Calculates 
  Eigen::Vector2f range( 10.0*sqrt(x*x + y*y), atan2f(y, x));
  return range;
}


void LocalizationModule::calculateMuBar(VectorMuf& mu_hat, VectorUtf& ut, VectorMuf& mu_bar) {
  //g(u_t, mu_(t-1)): Propagation step
  mu_bar = ball_loc_.A * mu_hat + ball_loc_.B * ut;
  // mu_bar = sin(mu_hat+ut); // Ficticious Nonlinear State Trasnsition
}

void LocalizationModule::calculateGandH(VectorMuf& mu_bar, MatrixAGf& A_or_G, MatrixCHf& C_or_H) {
// Get partial derivative of the state transition and observation function
  A_or_G = ball_loc_.A; // Linear Case for both KF and EKF

  // C_or_H = ball_loc_.C; // Linear Case for KF
  C_or_H << -mu_bar(2)/(mu_bar(0)*mu_bar(0) + mu_bar(2)*mu_bar(2)), 0.0, mu_bar(0)/(mu_bar(0)*mu_bar(0) + mu_bar(2)*mu_bar(2)), 0.0,
      10.0*mu_bar(0)/sqrt(mu_bar(0)*mu_bar(0) + mu_bar(2)*mu_bar(2)), 0.0, 10.0*mu_bar(2)/sqrt(mu_bar(0)*mu_bar(0) + mu_bar(2)*mu_bar(2)),0.0; // Nonlinear Case: bearing and distance measurements
}

void LocalizationModule::calculateMeasPred(VectorMuf& mu_bar, VectorZtf& z_bar) {
// CALCULATES h(mu_bar)
  
  // Linear x and y measurement for KF
  // z_bar = ball_loc_.C * mu_bar;

  // Nonlinear bearing and distance measurement for EKF
  z_bar << atan2f(mu_bar(2),mu_bar(0)),10.0*sqrt(mu_bar(0)*mu_bar(0) + mu_bar(2)*mu_bar(2));
}
