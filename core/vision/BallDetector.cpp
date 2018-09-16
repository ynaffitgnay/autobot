#include <vision/BallDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BallDetector::findBall() {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BALL, 300 },
  };
  static map<WorldObjectType,vector<int>> balls = {
    { WO_BALL, { 24, 15, 74, 83} }
  };
  auto fid = vblocks_.frame_info->frame_id;
  if(fid >= 6150) return;
  for(auto ball : balls) {
    auto& object = vblocks_.world_object->objects_[ball.first];
    auto box = ball.second;
    object.imageCenterX = (box[0] + box[2]) / 2;
    object.imageCenterY = (box[1] + box[3]) / 2;
    auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[ball.first]);
    object.visionDistance = cmatrix_.groundDistance(position);
    object.visionBearing = cmatrix_.bearing(position);
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
    tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(ball.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
}
