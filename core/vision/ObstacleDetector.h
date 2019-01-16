#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#pragma once

#include <vision/ObjectDetector.h>


class TextLogger;

/// @ingroup vision
class ObstacleDetector : public ObjectDetector {
 public:

  /*
  * default constructor. We did nothing
  */
  ObstacleDetector(DETECTOR_DECLARE_ARGS);

  /*
  * Logging function. We did nothing
  */
  void init(TextLogger* tl){ textlogger = tl; }

  /*
  * Acts kind of like a constructor because this is where we start changing things.
  * Set up of all variables, and calling of all functions. 
  * Also separates the blobs into color candidates.
  * This is intended to run each time process frames is called.
  * Input: Sorted list of blobs
  */
  void findObstacles(std::vector<Blob>& blobs);

 private:
  // Logger
  TextLogger* textlogger;

  void obsAssign(std::vector<Blob>& obstaclesCands);

  Point2D findCenter(Blob& blob);

  void addObstaclesObject(int newCenterX, int newCenterY, int width, int height, WorldObjectType wo_type);


  std::map<WorldObjectType,Pose2D> obstacles_ = {
    { WO_OBSTACLE_1, Pose2D(0.0, 1800.0, 750.0) },
    { WO_OBSTACLE_2, Pose2D(0.0, 750.0, 1350.0) }};

};

#endif
