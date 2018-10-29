#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/GoalDetector.h>
#include <vision/BeaconDetector.h>
#include <vision/LineDetector.h>
#include <vision/BallDetector.h>
#include <vision/Logging.h>
#include <vision/structures/Blob.h>
#include <common/ColorConversion.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera)
{
  enableCalibration_ = false;
  color_segmenter_ = std::make_unique<Classifier>(vblocks_, vparams_, iparams_, camera_);
  goal_detector_ = std::make_unique<GoalDetector>(DETECTOR_PASS_ARGS);
  ball_detector_ = std::make_unique<BallDetector>(DETECTOR_PASS_ARGS);
  beacon_detector_ = std::make_unique<BeaconDetector>(DETECTOR_PASS_ARGS);
  line_detector_ = std::make_unique<LineDetector>(DETECTOR_PASS_ARGS);
  calibration_ = std::make_unique<RobotCalibration>();
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  color_segmenter_->init(tl);
  goal_detector_ ->init(tl);
  ball_detector_->init(tl);
  beacon_detector_->init(tl);
  line_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

//void ImageProcessor::saveImg(std::string filepath) {
//  cv::Mat mat;
//  int xstep_ = 1 << iparams_.defaultHorizontalStepScale;
//  int ystep_ = 1 << iparams_.defaultVerticalStepScale;
//  cv::resize(color_segmenter_->img_grayscale(), mat, cv::Size(), 1.0 / xstep_, 1.0 / ystep_, cv::INTER_NEAREST); 
  
//  cv::imwrite(filepath, mat);
//}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_.data(), NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_.data(), NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_.data(), *abs_parts = vblocks_.body_model->abs_parts_.data();
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->isLoaded();
  return vblocks_.image->isLoaded();
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(const RobotCalibration& calibration){
  *calibration_ = calibration;
}

void ImageProcessor::processFrame(){
  BallCandidate* bestBall = nullptr;
  
  // clear out the blobs for a new frame
  std::vector<Blob>().swap(blobs_);



  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  tlog(30, "Process Frame camera %i", camera_);

  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;
  color_segmenter_->makeBlobs(blobs_);
  std::sort(blobs_.begin(), blobs_.end(), sortBlobAreaPredicate);

  if (camera_ == Camera::BOTTOM) {
    line_detector_->findPenaltyLine(blobs_);
  }

  if (vblocks_.world_object->objects_[WO_OWN_PENALTY].seen) {
    printf("Saw the penalty box\n");
  } else {
    // printf("Saw no penalty box\n");
  }

  // Populate world objects with the best ball candidate
  bestBall = getBestBallCandidate();
  if (bestBall) {
    // std::cout << "Ball distance: " << bestBall->visionDistance << ", Ball bearing: " << bestBall->visionBearing << std::endl;
  }

  beacon_detector_->findBeacons(blobs_);
  goal_detector_->findGoals(blobs_);


  // auto& ball = vblocks_.world_object->objects_[WO_BALL];
  // auto& goal = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
  // if(goal.seen){
  //   std::cout << "Goal distance: " << goal.visionDistance << ", Goal bearing: " << goal.visionBearing << ", Goal orientation: " << goal.orientation << "\t";
  // }
  // if(ball.seen){
  //   std::cout << "Ball distance: " << ball.visionDistance << ", Ball bearing: " << ball.visionBearing << std::endl;
  // }
}

int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  std::vector<BallCandidate*> ballCands = std::vector<BallCandidate*>();
  ball_detector_->findBall(blobs_, ballCands);

  return ballCands;
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  std::vector<BallCandidate*> ballCands = getBallCandidates();
  BallCandidate* bestCand = nullptr;
  // now put some heuristics in here to get the best one. for now just choose the first one.

  // Assuming that the bottom frame gets processed first.
  if (ballCands.size() == 0) {
    if (camera_ == Camera::BOTTOM) {
      vblocks_.world_object->objects_[WO_BALL].seen = false;
    } else if (camera_ == Camera::TOP && !vblocks_.world_object->objects_[WO_BALL].seen) {
      vblocks_.world_object->objects_[WO_BALL].seen = false;
    }
    return bestCand;
  }
  
  // FOR NOW: return the first one in the list
  bestCand = ballCands.at(0);

  auto& ball = vblocks_.world_object->objects_[WO_BALL];
  ball.imageCenterX = bestCand->centerX;
  ball.imageCenterY = bestCand->centerY;
  ball.radius = bestCand->radius;
  ball.visionDistance = bestCand->groundDistance;
  ball.visionBearing = bestCand->bearing;
  ball.visionElevation = bestCand->elevation;
  ball.seen = true;

  ball.fromTopCamera = (camera_ == Camera::TOP);
  tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_BALL), ball.imageCenterX, ball.imageCenterY, ball.visionDistance);

  return bestCand;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}
