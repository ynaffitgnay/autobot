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
  ball_loc_.Q << 5,0,
                0,5;

  ball_loc_.R << 10,0,0,0,
                0,10,0,0,
                0,0,10,0,
                0,0,0,10;

  ball_loc_.mu_hat << 0,0,0,0;

  ball_loc_.sig_hat << 10,0,0,0,
                      0,20,0,0,
                      0,0,10,0,
                      0,0,0,20;
  ball_loc_.A << 1,0.33,0,0,
                0,1,0,0,
                0,0,1,0.33,
                0,0,0,1;
  ball_loc_.B << 0,0,0,0,
                0,0,0,0,
                0,0,0,0,
                0,0,0,0;
  ball_loc_.C << 1,0,0,0,
                0,0,1,0;
  last_time_ = -1.0;
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

void LocalizationModule::processFrame() {
  double time = cache_.frame_info->seconds_since_start;
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  auto sloc = cache_.localization_mem->player_;
  self.loc = sloc;

  double dt = (last_time_ < 0) ? 1.0/30.0 : (time - last_time_);
  printf("dt = %.2f\t, last_time_ = %f\n",dt, last_time_);

  //TODO: modify this block to use your Kalman filter implementation
  ball_loc_.A << 1,dt,0,0,
                 0,1,0,0,
                 0,0,1,dt,
                 0,0,0,1;
  auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

  Eigen::Vector2f pos;
  pos = rangeToPos(ball.visionBearing,ball.visionDistance);

  Eigen::Vector4f ut(0,0,0,0);
  ekfilter_->runKF(ball_loc_.mu_hat, ball_loc_.sig_hat, ut, pos, ball_loc_.A, ball_loc_.B, ball_loc_.C, ball_loc_.Q, ball_loc_.R, ball.seen);
  // if(ball.seen) {
  //   // Compute the relative position of the ball from vision readings
  //   // ekfilter_->updateStep(ball_loc_.mu_bar, ball_loc_.sig_bar, ball_loc_.C, pos, z_bar, ball_loc_.Q, ball_loc_.mu_hat, ball_loc_.sig_hat);
    
  //   // Compute the global position of the ball based on our assumed position and orientation
    // ball.absVel = fill this in

  //   // Update the localization memory objects with localization calculations
  //   // so that they are drawn in the World window
  // } 
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
  // //TODO: How do we handle not seeing the ball?
  // else {
  //   ball_loc_.A << 1,dt,0,0,
  //                  0,1,0,0,
  //                  0,0,1,dt,
  //                  0,0,0,1;

  //   Eigen::Vector4f ut(0,0,0,0);
  //   ekfilter_->predictionStep(ball_loc_.mu_hat, ball_loc_.sig_hat, ut, ball_loc_.A, ball_loc_.B, ball_loc_.R, ball_loc_.mu_hat, ball_loc_.sig_hat);
  //   printf("No measurement: X hat = %.3f\t, Vx hat = %.3f\t, Y hat = %.3f\t, Vy hat = %.3f\n",ball_loc_.mu_hat(0),ball_loc_.mu_hat(1),ball_loc_.mu_hat(2),ball_loc_.mu_hat(3));
  //   printf("No measurement: Px = %.3f\t, Pvx = %.3f\t, Py = %.3f\t, Pvy = %.3f\n",ball_loc_.mu_hat(0),ball_loc_.mu_hat(1),ball_loc_.mu_hat(2),ball_loc_.mu_hat(3));
  //   // ball.distance = 10000.0f;
  //   // ball.bearing = 0.0f;
  // }
  last_time_ = cache_.frame_info->seconds_since_start;
}


Eigen::Vector2f LocalizationModule::rangeToPos(float bearing, float distance) {
  Eigen::Vector2f pos(distance*cos(bearing)/10.0,-distance*sin(bearing)/10.0);
  return pos;
}

Eigen::Vector2f LocalizationModule::posToRange(float x, float y) {
  Eigen::Vector2f range( sqrt(x*x + y*y), atan2f(y, x));
  return range;
}