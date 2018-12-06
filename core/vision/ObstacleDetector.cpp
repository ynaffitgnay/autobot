#include <vision/ObstacleDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

ObstacleDetector::ObstacleDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void ObstacleDetector::findObstacles(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  std::vector<Blob> obstaclesCands;
  std::vector<WorldObject> obstacles;
  for (int i = 0; i < blobs.size(); i++) {
    if(blobs.at(i).color == c_ORANGE) obstaclesCands.push_back(blobs.at(i));
  }
  obsAssign(obstaclesCands);
}

void ObstacleDetector::obsAssign(std::vector<Blob>& obstaclesCands) {
  for(auto& blob : obstaclesCands) {
    for( auto& obs : obstacles_){
      Point2D center = findCenter(blob);
      auto position = cmatrix_.getWorldPosition(center.x, center.y, 0.0);
      Point2D pos(position[0],position[1]);
      Point2D obsPt(obs.second.translation.x,obs.second.translation.y);
      float dist = obsPt.getDistanceTo(pos);
      if (dist < 150.0){
        addObstaclesObject(center.x, center.y, blob.xf - blob.xi, blob.yf - blob.yi, obs.first);
      }
    }
  }
}

Point2D ObstacleDetector::findCenter(Blob& blob){
  Point2D center;
  center.x = 0.5 * (blob.xi + blob.xf);
  center.y = 0.5 * (blob.yi + blob.yf);
  return center;
}


void ObstacleDetector::addObstaclesObject(int newCenterX, int newCenterY, int width, int height, WorldObjectType wo_type) {
  auto& obsObject = vblocks_.world_object->objects_[wo_type];
  obsObject.type = wo_type;
  obsObject.imageCenterX = newCenterX;
  obsObject.imageCenterY = newCenterY;
  auto position = cmatrix_.getWorldPosition(obsObject.imageCenterX, obsObject.imageCenterY, 0.0);
  obsObject.visionDistance = cmatrix_.groundDistance(position);
  obsObject.visionBearing = cmatrix_.bearing(position);
  obsObject.seen = true;
  obsObject.cwidth = width;
  obsObject.cheight = height;
  tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), obsObject.imageCenterX, obsObject.imageCenterY, obsObject.visionDistance);
}