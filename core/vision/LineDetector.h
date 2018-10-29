#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H

#pragma once

#include <vision/ObjectDetector.h>


class TextLogger;

/// @ingroup vision
class LineDetector : public ObjectDetector {
 public:

  /*
  * default constructor. We did nothing
  */
  LineDetector(DETECTOR_DECLARE_ARGS);

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
  void findPenaltyLine(std::vector<Blob>& blobs);
 private:
  // Logger
  TextLogger* textlogger;

  void detBestLine(std::vector<Blob> lineCand);

};

#endif
