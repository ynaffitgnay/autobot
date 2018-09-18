#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  std::map<WorldObjectType,int> heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
    { WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK, 200 },
    { WO_BEACON_PINK_BLUE, 200 }};
  std::vector<Blob> blueCand;
  std::vector<Blob> pinkCand;
  std::vector<Blob> yellowCand;
  for (int i = 0; i < blobs.size(); i++) {
    auto position = cmatrix_.getWorldPosition(blobs.at(i).avgX, blobs.at(i).avgY, 150); 
    auto visionDistance = cmatrix_.groundDistance(position);
    auto visionBearing = cmatrix_.bearing(position);
    std::cout << "Saw " << i << " at [" << blobs.at(i).avgX << " ," << blobs.at(i).avgY << "] with calculated distance " << visionDistance << std::endl;
    switch (blobs.at(i).color)
    {  
      case c_BLUE : 
        std::cout << "Pushed back blue" << std::endl;
        blueCand.push_back(blobs.at(i));
        break;
      case c_YELLOW :
        std::cout << "Pushed back yellow" << std::endl;
        yellowCand.push_back(blobs.at(i));
        break;
      case c_PINK :
        std::cout << "Pushed back pink" << std::endl;  
        pinkCand.push_back(blobs.at(i));
        break;
      default: // None of the beacon colors
        break;       
    }
  }

  genCombos(yellowCand,pinkCand,blueCand);
}

void BeaconDetector::genCombos(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs) {
  int height_thresh = 4;
  int width_thresh = 4;
  int xDiff;
  int yDiffBottom;
  int yDiffTop;
  int uniformTopBott;
  int uniformTopWhole;
  for (int i = 0; i < yBlobs.size(); i++) {
    for (int j = 0; j < bBlobs.size(); j++) {
      std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with blue at [" << bBlobs.at(j).avgX << " ," << bBlobs.at(j).avgY << "]" << std::endl;
      xDiff = std::abs(yBlobs.at(i).avgX - bBlobs.at(j).avgX); // Centers align
      yDiffBottom = bBlobs.at(j).yf - yBlobs.at(i).yi;  // yellow on bottom touches blue on top
      yDiffTop = yBlobs.at(i).yf - bBlobs.at(j).yi; // yellow on top touches blue on bottom 
      uniformTopBott = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).widthEnd); // top width is the same as dx approximately
      uniformTopWhole = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).dx);
      // 
      if ( xDiff < std::round(yBlobs.at(i).total*0.1) ) {
        std::cout << "Xdiff within threshold" << std::endl;
        if (yDiffBottom < height_thresh) {
          addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yi, WO_BEACON_BLUE_YELLOW);
        }
        if (yDiffTop < height_thresh) {          
          addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yf, WO_BEACON_YELLOW_BLUE);    
        }
        if (uniformTopBott < width_thresh) {
          std::cout << "3 width is uniform from top to bottom" << std::endl;
        }
        if (uniformTopWhole < width_thresh) {
          std::cout << "4 width is uniform comparing top to whole" << std::endl;
        }
      }
    }
    std::cout << "Comparing pink now" << std::endl;
    for (int k = 0; k < pBlobs.size(); k++) {
      std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with pink at [" << pBlobs.at(k).avgX << " ," << pBlobs.at(k).avgY << "]" << std::endl;
      xDiff = std::abs(yBlobs.at(i).avgX - pBlobs.at(k).avgX); // Centers align
      yDiffBottom = pBlobs.at(k).yf - yBlobs.at(i).yi;  // yellow on bottom touches blue on top
      yDiffTop = yBlobs.at(i).yf - pBlobs.at(k).yi; // yellow on top touches blue on bottom 
      uniformTopBott = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).widthEnd); // top width is the same as dx approximately
      uniformTopWhole = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).dx);
      // 
      if ( xDiff < std::round(yBlobs.at(i).total*0.1) ) {
        std::cout << "Xdiff within threshold" << std::endl;
        if (yDiffBottom < height_thresh) {
          std::cout << "1 yDiffBottom within threshold, pink is just on top of yellow" << std::endl;
          addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yi, WO_BEACON_PINK_YELLOW);
        }
        if (yDiffTop < height_thresh) {
          std::cout << "2 yDiffTop within threshold, yellow is just on top of pink" << std::endl;
          addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yf, WO_BEACON_YELLOW_PINK);
        }
        if (uniformTopBott < width_thresh) {
          std::cout << "3 width is uniform from top to bottom" << std::endl;
        }
        if (uniformTopWhole < width_thresh) {
          std::cout << "4 width is uniform comparing top to whole" << std::endl;
        }
      }
    }
  }
  for (int p = 0; p < bBlobs.size(); p++) { 
    for (int q = 0; q < pBlobs.size(); q++) {
        std::cout << "Comparing blue at [" << bBlobs.at(p).avgX << " ," << bBlobs.at(p).avgY << "] with pink at [" << pBlobs.at(q).avgX << " ," << pBlobs.at(q).avgY << "]" << std::endl;
        xDiff = std::abs(bBlobs.at(p).avgX - pBlobs.at(q).avgX); // Centers align
        yDiffBottom = pBlobs.at(q).yf - bBlobs.at(p).yi;  // yellow on bottom touches blue on top
        yDiffTop = bBlobs.at(p).yf - pBlobs.at(q).yi; // yellow on top touches blue on bottom 
        uniformTopBott = std::abs(bBlobs.at(p).widthStart - bBlobs.at(p).widthEnd); // top width is the same as dx approximately
        uniformTopWhole = std::abs(bBlobs.at(p).widthStart - bBlobs.at(p).dx);
        // 
        if ( xDiff < std::round(bBlobs.at(p).total*0.1) ) {
          std::cout << "Xdiff within threshold" << std::endl;
          if (yDiffBottom < height_thresh) {
            std::cout << "1 yDiffBottom within threshold, pink is just on top of blue" << std::endl;
            addBeaconObject(bBlobs.at(p).avgX, bBlobs.at(p).yi, WO_BEACON_PINK_BLUE);
          }
          if (yDiffTop < height_thresh) {
            std::cout << "2 yDiffTop within threshold, blue is just on top of pink" << std::endl;
            addBeaconObject(bBlobs.at(p).avgX, bBlobs.at(p).yf, WO_BEACON_BLUE_PINK);
          }
          if (uniformTopBott < width_thresh) {
            std::cout << "3 width is uniform from top to bottom" << std::endl;
          }
          if (uniformTopWhole < width_thresh) {
            std::cout << "4 width is uniform comparing top to whole" << std::endl;
          }
        }
      }
    }
}

void BeaconDetector::addBeaconObject(int newCenterX,int newCenterY,WorldObjectType wo_type) {
  auto& object = vblocks_.world_object->objects_[wo_type];
  object.imageCenterX = newCenterX;
  object.imageCenterY = newCenterY;
  auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights_[wo_type]);
  object.visionDistance = cmatrix_.groundDistance(position);
  object.visionBearing = cmatrix_.bearing(position);
  object.seen = true;
  object.fromTopCamera = camera_ == Camera::TOP;
  tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), object.imageCenterX, object.imageCenterY, object.visionDistance);
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
