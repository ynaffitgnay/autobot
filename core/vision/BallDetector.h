#ifndef BALL_DETECTOR_H
#define BALL_DETECTOR_H
#pragma once

#include <vision/ObjectDetector.h>
#include <vision/structures/Blob.h>

class TextLogger;

/// @ingroup vision
class BallDetector : public ObjectDetector {
 public:
  BallDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void detectBall(std::vector<Blob>& blobs);
  bool findBall2(std::vector<Blob>& blobs, int& imageX, int& imageY);
  void findBall(std::vector<Blob>& blobs);
 private:
  unsigned char* getSegImg();
  TextLogger* textlogger;
};

#endif
