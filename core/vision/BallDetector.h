#pragma once

#include <vision/ObjectDetector.h>

class TextLogger;

/// @ingroup vision
class BallDetector : public ObjectDetector {
 public:
  BallDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBall();
 private:
  TextLogger* textlogger;
};
