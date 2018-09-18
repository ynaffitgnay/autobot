#ifndef BEACON_DETECTOR_H
#define BEACON_DETECTOR_H

#pragma once

#include <vision/ObjectDetector.h>

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  void findBeacons(std::vector<Blob>& blobs);
 private:
  TextLogger* textlogger;
  void genCombos(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs,
                 std::vector<WorldObject*>& yb_beacons, std::vector<WorldObject*>& by_beacons,
                 std::vector<WorldObject*>& yp_beacons, std::vector<WorldObject*>& py_beacons,
                 std::vector<WorldObject*>& bp_beacons, std::vector<WorldObject*>& pb_beacons );
  WorldObject addBeaconObject(int newCenterX,int newCenterY,WorldObjectType wo_type);
  bool alignsX(Blob& blobA, Blob& blobB);
  int alignsY(Blob& blobA, Blob& blobB);
  void chooseBestBeacons(std::vector<std::vector<WorldObject*> > beacon_list);
  std::map<WorldObjectType,int> heights_ = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 }};

};

#endif
