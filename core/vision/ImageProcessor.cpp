#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/BeaconDetector.h>
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
  beacon_detector_ = std::make_unique<BeaconDetector>(DETECTOR_PASS_ARGS);
  calibration_ = std::make_unique<RobotCalibration>();
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  color_segmenter_->init(tl);
  beacon_detector_->init(tl);
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
  std::vector<Blob> blobs;
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  tlog(30, "Process Frame camera %i", camera_);

  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;
  color_segmenter_->makeBlobs(blobs);
  detectBall(blobs);
  detectGoal();
  beacon_detector_->findBeacons();
}

void ImageProcessor::detectBall(std::vector<Blob>& blobs) {
  int imageX, imageY;
  if(!findBall(blobs, imageX,imageY)) return; // findBall fills in imageX and imageY
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  // Now we know where the ball's position
  ball->seen = true;

}

// Puts the center of the ball in imageX and imageY
bool ImageProcessor::findBall(std::vector<Blob>& blobs, int& imageX, int& imageY) {
  if(camera_ == Camera::TOP) {
    // TODO: fix this logic (right now we only detect one orange ball)
    Blob orangeBlob;
    for (auto blob : blobs) {
      if (blob.color == c_ORANGE) {
        orangeBlob = blob;
        break;
      }
    }

    if (orangeBlob.color != c_ORANGE) {
      return false;
    }
    cv::Mat frame, grayFrame;

    // Make sure the row/col you want are within range
    int row = ((orangeBlob.yi - 50) < 0) ? 0 : (orangeBlob.yi - 50);
    int col = ((orangeBlob.xi - 50) < 0) ? 0 : (orangeBlob.xi - 50);
    int width = ((2 * (orangeBlob.xf - orangeBlob.xi)) > iparams_.width) ? iparams_.width : (2 *(orangeBlob.xf - orangeBlob.xi));
    int height = ((orangeBlob.yf - orangeBlob.yi + 100) > iparams_.height) ? iparams_.height : (orangeBlob.yf - orangeBlob.yi + 100);
    
    //std::cout << "row: " << row << " col: " << col << " width: " << width << " height: " << height << "\n";
  
    //frame = color::rawToMatSubset(vblocks_.image->getImgTop(), iparams_, row, col, width, height, 1, 1);
    grayFrame = color::rawToMatGraySubset(vblocks_.image->getImgTop(), iparams_, row, col, width, height, 1, 1);
    
    std::vector<cv::Vec3f> circles;
  
    if (!grayFrame.data) {
      return false;
    }
    
    // Check to see if getting rid of this speeds anything up
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(9, 9), 2, 2);

    cv::HoughCircles(grayFrame, circles, CV_HOUGH_GRADIENT, 30, grayFrame.rows/8, 50, 100, 0, 0);
    
    // Now turn circles into a vector of floats
    // Populates a v with circles.size() elements, each a vector with 3 floats
    std::vector<std::vector<float>> v(circles.size(), std::vector<float>(3));
  
    for (size_t i = 0; i < circles.size(); ++i) {
      const cv::Vec3f& c = circles[i];
      v[i][0] = c[0] + col; // if downsampled
      v[i][1] = c[1] + row; // if downsampled
      v[i][2] = c[2];
  
      std::cout << "Circle " << i << " x: " << v[i][0] << " y: " << v[i][1] << " r: " << v[i][2] << "\n";
    }
  }
  
  imageX = imageY = 0;
  int total = 0;
  float x_mean, y_mean;
  x_mean = y_mean = 0.0;

  //std::cout << "iparams_.width: " << iparams_.width << " iparams_.height: " << iparams_.height << "\n";
  
  // Process from left to right
  for(int x = 0; x < iparams_.width; x++) {
    // Process from top to bottom
    for(int y = 0; y < iparams_.height; y++) {
      // Retrieve the segmented color of the pixel at (x,y)
      auto c = getSegImg()[y * iparams_.width + x];
      if(c == c_ORANGE){
        total++;
        x_mean = x_mean + x;
        y_mean = y_mean + y;
      }
    }
  }
  
  if(total <= 20){
    return false;
  }
  x_mean = x_mean/total;
  y_mean = y_mean/total;
  imageX = (int)x_mean;
  imageY = (int)y_mean;

  std::cout << "ImageX: " << imageX << " ImageY: " << imageY << "\n";
  
  // TO DO: Add heuristics to return false if ball detection fails
  return true;
}

void ImageProcessor::detectGoal() {
  int imageX, imageY;
  if(!findGoal(imageX,imageY)) return; // findBall fills in imageX and imageY
  WorldObject* goal = &vblocks_.world_object->objects_[WO_OWN_GOAL];

  goal->imageCenterX = imageX;
  goal->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  goal->visionBearing = cmatrix_.bearing(p);
  goal->visionElevation = cmatrix_.elevation(p);
  goal->visionDistance = cmatrix_.groundDistance(p);
  // Now we know where the goal's position
  goal->seen = true;

}

bool ImageProcessor::findGoal(int& imageX, int& imageY) {
  imageX = imageY = 0;
  int total = 0;
  float x_mean, y_mean;
  x_mean = y_mean = 0.0;
  // Process from left to right
  for(int x = 0; x < iparams_.width; x++) {
    // Process from top to bottom
    for(int y = 0; y < iparams_.height; y++) {
      // Retrieve the segmented color of the pixel at (x,y)
      auto c = getSegImg()[y * iparams_.width + x];
      if(c == c_BLUE){
        total++;
        x_mean = x_mean + x;
        y_mean = y_mean + y;
      }
    }
  }

  if(total <= 100){
    return false;
  }
  x_mean = x_mean/total;
  y_mean = y_mean/total;
  imageX = (int)x_mean;
  imageY = (int)y_mean;

  // TO DO: Add heuristics to return false if ball detection fails
  return true;
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
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}
