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
  printf("Num Obstacle Cands: %d\n",obstaclesCands.size());
  auto& obs1 = vblocks_.world_object->objects_[WO_OBSTACLE_1];
  auto& obs2 = vblocks_.world_object->objects_[WO_OBSTACLE_2];
  obs1.width = 600.0;
  obs1.length = 300.0;
  obs2.width = 600.0;
  obs2.length = 300.0;
  obsAssign(obstaclesCands);
}

void ObstacleDetector::obsAssign(std::vector<Blob>& obstaclesCands) {
  for(auto& blob : obstaclesCands) {
    for( auto& obs : obstacles_){
      Point2D center = findCenter(blob);
      // printf("Image center of blob [%f, %f]\n", center.x, center.y);
      auto position = cmatrix_.getWorldPosition(center.x, center.y, 0.0);
      Point2D relPos(position[0],position[1]);
      // printf("Relative position of blob center to robot [%f, %f]\n", relPos.x, relPos.y);
      Point2D obsPt(obs.second.translation.x,obs.second.translation.y);
      // printf("Comparing with obstacle: %s at [%f, %f]\n",getName(obs.first), obsPt.x, obsPt.y);
      float dist = obsPt.getDistanceTo(relPos);
      // printf("Estimated distance: %f\n", dist);
      // printf("Blob size: %d Blob width: %d Blob height: %d \n", blob.total, blob.xf-blob.xi, blob.yf-blob.yi);
      if (blob.total > 2000){
        addObstaclesObject(center.x, center.y, blob.xf - blob.xi, blob.yf - blob.yi, WO_OBSTACLE_UNKNOWN);
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
  printf("Obstacle at [%f, %f]\n", position.x, position.y);
  tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), obsObject.imageCenterX, obsObject.imageCenterY, obsObject.visionDistance);
}