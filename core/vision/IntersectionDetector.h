#ifndef INTERSECTION_DETECTOR_H
#define INTERSECTION_DETECTOR_H

#pragma once

#include <vision/ObjectDetector.h>


class TextLogger;

/// @ingroup vision
class IntersectionDetector : public ObjectDetector {
 public:

  /*
  * default constructor. We did nothing
  */
  IntersectionDetector(DETECTOR_DECLARE_ARGS);

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
  void findIntersections(std::vector<Blob>& blobs);
 private:
  // Logger
  TextLogger* textlogger;

  /*
  * Main function for converting blobs into intersections. 
  * Cycles through the colors and calls other helper functions. 
  * Input: Candidate blobs.
  * Output: A list of possible intersections for each combination of colors.
  */
  void processBlobs(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs,
                 std::vector<WorldObject>& y_intersections, std::vector<WorldObject>& p_intersections );

  bool neighbors(Blob& blobA, Blob& blobB);

  /*
  * Function for converting a set of blobs into a intersection world object.
  * This is not the final intersection, but simply a candidate. 
  * Input: XY coords of new intersection and intersection type.
  * Output: WorldObject of the same intersection type.
  */
  WorldObject addIntersectionObject(int newCenterX,int newCenterY,float aspectRatio);


  /*
  * Function for discriminating between potential intersections of the same type.
  * Compares fitness of each blob of the same type to itself and chooses the best candidate for each type.
  * Also looks at how close two intersections are together to eliminate stacked blobs from being counted as intersections. 
  * The actual world objects are sent out in this function.
  * Input: A vector of all of the candidates for each intersection type
  */
  void chooseBestIntersections(std::vector<WorldObject>& y_intersections,std::vector<WorldObject>& p_intersections);

  /*
  * All cards are 65mmx75mm individually
  */
  std::map<WorldObjectType,int> aspectRatios_ = {
    { WO_OPP_PEN_LEFT_L, 3 },
    { WO_OPP_PEN_RIGHT_L, 3 },
    { WO_OPP_PEN_LEFT_T, 2 },
    { WO_OPP_PEN_RIGHT_T, 2 },
    { WO_OWN_PEN_RIGHT_T, 4 },
    { WO_OWN_PEN_LEFT_T, 4 }};

};

#endif
