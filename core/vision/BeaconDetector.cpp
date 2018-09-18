#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 }
  };

  std::vector<Blob> blueCand;
  std::vector<Blob> pinkCand;
  std::vector<Blob> yellowCand;
  for (int i; i < blobs.size(); i++) {
    auto position = cmatrix_.getWorldPosition(blobs.at(i).avgX, blobs.at(i).avgY, 150);
    auto visionDistance = cmatrix_.groundDistance(position);
    auto visionBearing = cmatrix_.bearing(position);
    std::cout << "Saw " << i << " at [" << blobs.at(i).avgX << " ," << blobs.at(i).avgY << "] with calculated distance " << visionDistance << std::endl;
    switch (blobs.at(i).color)
    {  
      case c_BLUE :  

        blueCand.push_back(blobs.at(i));
      case c_YELLOW :
        yellowCand.push_back(blobs.at(i));
      case c_PINK :  
        pinkCand.push_back(blobs.at(i));
      break;
      default: // None of the beacon colors
      break;       
    }
  }
}

void BeaconDetector::genCombos(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs) {
  for (int i; i < yBlobs.size(); i++) {
    for (int j; i < bBlobs.size(); j++) {
      int xDiff = std::abs(yBlobs.at(i).avgX - bBlobs.at(j).avgX);
      int yDiffBottom = bBlobs.at(j).yf - yBlobs.at(i).yi;
      int yDiffTop = yBlobs.at(i).yf - bBlobs.at(j).yi;
      if ( true) {
      }
    }
    for (int k; k < pBlobs.size(); k++) {
    }
  }
}





//   static map<WorldObjectType,vector<int>> beacons = {
//     { WO_BEACON_YELLOW_BLUE, { 24, 15, 74, 83} },
//     { WO_BEACON_YELLOW_PINK, { 104, 41, 138, 96 } },
//     { WO_BEACON_PINK_YELLOW, { 187, 38, 212, 90 } },
//     { WO_BEACON_BLUE_PINK, { 246, 36, 268, 86 } }
//   };
//   auto fid = vblocks_.frame_info->frame_id;
//   if(fid >= 6150) return;
//   for(auto beacon : beacons) {
//     auto& object = vblocks_.world_object->objects_[beacon.first];
//     auto box = beacon.second;
//     object.imageCenterX = (box[0] + box[2]) / 2;
//     object.imageCenterY = (box[1] + box[3]) / 2;
//     auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
//     object.visionDistance = cmatrix_.groundDistance(position);
//     object.visionBearing = cmatrix_.bearing(position);
//     object.seen = true;
//     object.fromTopCamera = camera_ == Camera::TOP;
//     tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
//   }
// }
