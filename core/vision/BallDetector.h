#ifndef BALL_DETECTOR_H
#define BALL_DETECTOR_H
#pragma once

#include <vision/ObjectDetector.h>
#include <vision/structures/Blob.h>
#include <vision/structures/BallCandidate.h>

class TextLogger;

/// @ingroup vision
class BallDetector : public ObjectDetector {
 public:
  BallDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBall(std::vector<Blob>& blobs, std::vector<BallCandidate*>& ballCands);
  bool checkBottomColor(Blob * orangeBlob);
  bool checkSideColors(Blob * orangeBlob);
  bool checkNextToLine(Blob * orangeBlob);
 private:
  unsigned char* getImg();
  unsigned char* getSegImg();
  TextLogger* textlogger;
};

#endif
