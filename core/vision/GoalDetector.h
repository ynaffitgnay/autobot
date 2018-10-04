#ifndef GOAL_DETECTOR_H
#define GOAL_DETECTOR_H

#pragma once

#include <vector>
#include <vision/ObjectDetector.h>
#include <vision/structures/Blob.h>

class TextLogger;

/// @ingroup vision
class GoalDetector : public ObjectDetector {
 public:
  GoalDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findGoals(std::vector<Blob>& blobs);
  unsigned char* getSegImg();
 private:
  bool goalSkewedPos(Blob* goalBlob);
  TextLogger* textlogger;
};
#endif
