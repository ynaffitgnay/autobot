#pragma once

#include <vision/ObjectDetector.h>

class TextLogger;

/// @ingroup vision
class GoalDetector : public ObjectDetector {
 public:
  GoalDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findGoals();
 private:
  TextLogger* textlogger;
};
