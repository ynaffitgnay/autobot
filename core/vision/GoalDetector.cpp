#include <vision/GoalDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>
#include <math/Common.h>

using namespace Eigen;
using namespace static_math;

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void GoalDetector::findGoals(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_UNKNOWN_GOAL, 253 }
  };
  float b = 330;
  // auto fid = vblocks_.frame_info->frame_id;
  // if(fid >= 6150) return;
  if((blobs.at(0).color == c_BLUE) && (blobs.at(0).lpCount>=10) && (blobs.at(0).total>=500)) { // Already 
    float correctaspectRatio = 1.8;
    auto& object = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    object.imageCenterY = blobs.at(0).avgY;
    float aspectRatio = ((float) blobs.at(0).dx)/((float) blobs.at(0).dy);
    // std::cout << "Goal Found!" << std::endl;
    // printf("Width = %d, Height = %d, Width/Height = %f\n", blobs.at(0).dx,  blobs.at(0).dy, ((float)blobs.at(0).dx)/((float)blobs.at(0).dy));
    // If not skewed
    if (aspectRatio>1.6 && aspectRatio<1.9)
    {
      object.orientation = 0.0;
      object.imageCenterX = blobs.at(0).avgX;
      auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[WO_UNKNOWN_GOAL]);
      object.visionDistance = cmatrix_.groundDistance(position);
      object.visionBearing = cmatrix_.bearing(position);
      object.seen = true;
      object.fromTopCamera = camera_ == Camera::TOP;
    }
    // else skewed
    else {
      object.orientation = acosf(aspectRatio/correctaspectRatio);
      float s_theta = sqrtf(1.0 - aspectRatio * aspectRatio / correctaspectRatio / correctaspectRatio);
      auto lColor = getSegImg()[(blobs.at(0).yf-5)*iparams_.width + blobs.at(0).xi-5];
      auto rColor = getSegImg()[(blobs.at(0).yf-5)*iparams_.width + blobs.at(0).xf-5];
      auto pos = cmatrix_.getWorldPosition(blobs.at(0).avgX,blobs.at(0).avgY, heights[WO_UNKNOWN_GOAL]);
      float b_theta = cmatrix_.getCameraWidthByDistance(cmatrix_.groundDistance(pos),b*s_theta);
      // std::cout << "b sin(theta) into image frame: " << b_theta << std::endl;
      if (lColor == c_FIELD_GREEN)
      {
        object.imageCenterX = blobs.at(0).avgX + b_theta;
      }
      else {
        object.imageCenterX = blobs.at(0).avgX - b_theta;
      }
      auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[WO_UNKNOWN_GOAL]);
      object.visionDistance = cmatrix_.groundDistance(position);
    }
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
    std::cout << "Distance Goal: " << object.visionDistance << " center X: " << object.imageCenterX << " center Y: " << object.imageCenterY << std::endl;
    tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_UNKNOWN_GOAL), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
  else {
    auto& object = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    object.seen = false;
    // std::cout << "No Goal here!" << std::endl;
  }
}

unsigned char* GoalDetector::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

// static map<WorldObjectType,vector<int>> goals = {
//   { WO_UNKNOWN_GOAL, { 24, 15, 74, 83} }
// };
//   for(auto goal : goals) {
//     auto& object = vblocks_.world_object->objects_[goal.first];
//     auto box = goal.second;
//     object.imageCenterX = (box[0] + box[2]) / 2;
//     object.imageCenterY = (box[1] + box[3]) / 2;
//     auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[goal.first]);
//     object.visionDistance = cmatrix_.groundDistance(position);
//     object.visionBearing = cmatrix_.bearing(position);
//     object.seen = true;
//     object.fromTopCamera = camera_ == Camera::TOP;
//     tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(goal.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
//   }
// }