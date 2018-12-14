#include <vision/ObstacleDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>
#include <math/Geometry.h>

using namespace Eigen;

ObstacleDetector::ObstacleDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void ObstacleDetector::findObstacles(std::vector<Blob>& blobs) {
  if(camera_ == Camera::TOP) return;
  std::vector<Blob> obstaclesCands;
  std::vector<WorldObject> obstacles;
  for (int i = 0; i < blobs.size(); i++) {
    if(blobs.at(i).color == c_ORANGE) obstaclesCands.push_back(blobs.at(i));
  }
  // printf("Num Obstacle Cands: %d\n",obstaclesCands.size());
  // auto& obs1 = vblocks_.world_object->objects_[WO_OBSTACLE_1];
  // auto& obs2 = vblocks_.world_object->objects_[WO_OBSTACLE_2];
  // obs1.width = 600.0;
  // obs1.length = 300.0;
  // obs2.width = 600.0;
  // obs2.length = 300.0;
  obsAssign(obstaclesCands);
}

void ObstacleDetector::obsAssign(std::vector<Blob>& obstaclesCands) {
  int count = 0;
  for(auto& blob : obstaclesCands) {
      Point2D center = findCenter(blob);
      // // printf("Image center of blob [%f, %f]\n", center.x, center.y);
      // auto position = cmatrix_.getWorldPosition(center.x, center.y, 0.0);
      // Point2D relPos(position[0],position[1]);
      // // printf("Relative position of blob center to robot [%f, %f]\n", relPos.x, relPos.y);
      // Point2D obsPt(obs.second.translation.x,obs.second.translation.y);
      // // printf("Comparing with obstacle: %s at [%f, %f]\n",getName(obs.first), obsPt.x, obsPt.y);
      // float dist = obsPt.getDistanceTo(relPos);
      // printf("Estimated distance: %f\n", dist);
      printf("Blob size: %d Blob width: %d Blob height: %d \n", blob.total, blob.xf-blob.xi, blob.yf-blob.yi);
      if (blob.total > 6000){
        addObstaclesObject(center.x, center.y, blob.xf - blob.xi, blob.yf - blob.yi, WO_OBSTACLE_UNKNOWN_1);
        count++;
      }
  }
  std::cout << "Detected " << count << " obstacles in this frame." << std::endl;
}

Point2D ObstacleDetector::findCenter(Blob& blob){
  Point2D center;
  center.x = 0.5 * (blob.xi + blob.xf);
  center.y = 0.5 * (blob.yi + blob.yf);
  return center;
}


void ObstacleDetector::addObstaclesObject(int newCenterX, int newCenterY, int width, int height, WorldObjectType wo_type) {
  auto& obsObject = vblocks_.world_object->objects_[wo_type];
  auto& robo = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
  auto& plan = vblocks_.planning;

  obsObject.type = wo_type;
  obsObject.imageCenterX = newCenterX;
  obsObject.imageCenterY = newCenterY;
  auto position = cmatrix_.getWorldPosition(obsObject.imageCenterX, obsObject.imageCenterY, 0.0);
  obsObject.relPos = Point2D(position[0], position[1]);
  obsObject.visionDistance = cmatrix_.groundDistance(position);
  obsObject.visionBearing = cmatrix_.bearing(position);
  obsObject.seen = true;
  obsObject.cwidth = width;
  obsObject.cheight = height;
  // TODO: get the gridIdx of this global value and then check if the cell is known to be occupied
  // if not, can marked plan->changedCost
  // In PlanningModule, can do a BFS search for "changed" nodes around current location to replan
  obsObject.loc = obsObject.relPos.relativeToGlobal(robo.loc, robo.orientation);
  obsObject.fromTopCamera = camera_ == Camera::TOP;

  // For now, can assume that the detected obstacle is in the next cell in the path
  auto& obsCell = plan->grid.at(plan->path[plan->pathIdx]);
  if (!obsCell.occupied) {
    obsCell.occupied = true; 
    // Since this cell was originally on our path, we know that the occupancy of this cell has changed
    plan->changedCost = true;
  } else {
    std::cout << "Unexpectedly discovered previously-known obstacle" << std::endl;
  }
  
  printf("\n\nObstacle at [%f, %f]\n\n\n", obsObject.loc.x, obsObject.loc.y);
  // tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), obsObject.imageCenterX, obsObject.imageCenterY, obsObject.visionDistance);
}
