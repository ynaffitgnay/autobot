#ifndef BEACON_DETECTOR_H
#define BEACON_DETECTOR_H

#pragma once

#include <vision/ObjectDetector.h>


class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:

  /*
  * default constructor. We did nothing
  */
  BeaconDetector(DETECTOR_DECLARE_ARGS);

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
  void findBeacons(std::vector<Blob>& blobs);
 private:
  // Logger
  TextLogger* textlogger;

  /*
  * Main function for converting blobs into beacons. 
  * Cycles through the colors and calls other helper functions. 
  * Input: Candidate blobs.
  * Output: A list of possible beacons for each combination of colors.
  */
  void genCombos(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs, std::vector<Blob>& wBlobs,
                 std::vector<WorldObject>& yb_beacons, std::vector<WorldObject>& by_beacons,
                 std::vector<WorldObject>& yp_beacons, std::vector<WorldObject>& py_beacons,
                 std::vector<WorldObject>& bp_beacons, std::vector<WorldObject>& pb_beacons );

  /*
  * Function for converting a set of blobs into a beacon world object.
  * This is not the final beacon, but simply a candidate. 
  * Input: XY coords of new beacon and beacon type.
  * Output: WorldObject of the same beacon type.
  */
  WorldObject addBeaconObject(int newCenterX,int newCenterY,WorldObjectType wo_type);

  /*
  * Helper function for checking the vertical fitness of both blobs for potential beaconhood
  * Input: A set of blobs to compare
  * Output: Aligned or not
  */
  bool alignsX(Blob& blobA, Blob& blobB);

  /*
  * Helper function for checking the horizontal fitness of both blobs for potential beaconhood
  * Input: A set of blobs to compare
  * Output: Above (1), Below (-1), Way far away and discountable (0)
  */
  int alignsY(Blob& blobA, Blob& blobB);

  /*
  * Function for examining if the object makes sense in the world when looking at the camera matrix
  * Input: Beacon object and its component blobs.
  * Output: Probably vs probably not in the world
  * 
  */
  bool objectValidInWorld(WorldObject& object, Blob& topBlob, Blob& bottomBlob);

  /*
  * Function for discriminating between potential beacons of the same type.
  * Compares fitness of each blob of the same type to itself and chooses the best candidate for each type.
  * Also looks at how close two beacons are together to eliminate stacked blobs from being counted as beacons. 
  * The actual world objects are sent out in this function.
  * Input: A vector of all of the candidates for each beacon type
  */
  void chooseBestBeacons(std::vector<std::vector<WorldObject> >& beacon_list);

  /*
  * Just a place to store the real heights
  */
  std::map<WorldObjectType,int> heights_ = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 }};

};

#endif
