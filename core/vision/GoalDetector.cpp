#include <vision/GoalDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void GoalDetector::findGoals(std::vector<Blob>& blobs) {
  std::cout << "Starting FindGoals!" << std::endl;
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_UNKNOWN_GOAL, 325 }
  };
  // auto fid = vblocks_.frame_info->frame_id;
  // if(fid >= 6150) return;
  if(blobs.at(0).color == c_BLUE) {
    auto& object = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    object.imageCenterX = blobs.at(0).avgX;
    object.imageCenterY = blobs.at(0).avgY;
    printf("WidthStart = %d, WidthEnd = %d\n", blobs.at(0).widthStart, blobs.at(0).widthEnd);
    auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[WO_UNKNOWN_GOAL]);
    object.visionDistance = cmatrix_.groundDistance(position);
    object.visionBearing = cmatrix_.bearing(position);
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
    std::cout << "Goal Found!" << std::endl;
    tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_UNKNOWN_GOAL), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
  else {
    auto& object = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    object.seen = false;
    std::cout << "No Goal here!" << std::endl;
  }
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