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
  std::vector<WorldObject*> yb_beacons;
  std::vector<WorldObject*> by_beacons;
  std::vector<WorldObject*> yp_beacons;
  std::vector<WorldObject*> py_beacons;
  std::vector<WorldObject*> bp_beacons;
  std::vector<WorldObject*> pb_beacons;
  std::vector<std::vector<WorldObject*> > beacon_list;
  for (int i = 0; i < blobs.size(); i++) {
    auto position = cmatrix_.getWorldPosition(blobs.at(i).avgX, blobs.at(i).avgY, 150); 
    auto visionDistance = cmatrix_.groundDistance(position);
    auto visionBearing = cmatrix_.bearing(position);
    std::cout << "Saw " << i << " at [" << blobs.at(i).avgX << " ," << blobs.at(i).avgY << "] with calculated distance " << visionDistance << std::endl;
    switch (blobs.at(i).color)
    {  
      case c_BLUE : 
        // std::cout << "Pushed back blue" << std::endl;
        blueCand.push_back(blobs.at(i));
        break;
      case c_YELLOW :
        // std::cout << "Pushed back yellow" << std::endl;
        yellowCand.push_back(blobs.at(i));
        break;
      case c_PINK :
        // std::cout << "Pushed back pink" << std::endl;  
        pinkCand.push_back(blobs.at(i));
        break;
      default: // None of the beacon colors
        break;       
    }
  }

  genCombos(yellowCand,pinkCand,blueCand,yb_beacons,by_beacons,yp_beacons,py_beacons,bp_beacons,pb_beacons);
  std::cout << "YB beacons: " << yb_beacons.size() << " BY beacons: " << by_beacons.size() << " YP beacons: " << yp_beacons.size() << " PY beacons: " << py_beacons.size() << " BP beacons: " << bp_beacons.size() << " PB beacons: " << pb_beacons.size() << std::endl;
  
  beacon_list.push_back(yb_beacons);
  beacon_list.push_back(by_beacons);
  beacon_list.push_back(yp_beacons);
  beacon_list.push_back(py_beacons);
  beacon_list.push_back(bp_beacons);
  beacon_list.push_back(pb_beacons);
  chooseBestBeacons(beacon_list);
}

void BeaconDetector::genCombos(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs,
                               std::vector<WorldObject*>& yb_beacons, std::vector<WorldObject*>& by_beacons,
                               std::vector<WorldObject*>& yp_beacons, std::vector<WorldObject*>& py_beacons,
                               std::vector<WorldObject*>& bp_beacons, std::vector<WorldObject*>& pb_beacons ) {
  int height_thresh = 4;
  int width_thresh = 4;
  int xDiff;
  int yDiffBottom;
  int yDiffTop;
  int uniformTopBott;
  int uniformTopWhole;
  WorldObject object;
  for (int i = 0; i < yBlobs.size(); i++) {
    for (int j = 0; j < bBlobs.size(); j++) {
      // std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with blue at [" << bBlobs.at(j).avgX << " ," << bBlobs.at(j).avgY << "]" << std::endl;
      yDiffBottom = bBlobs.at(j).yf - yBlobs.at(i).yi;  // yellow on bottom touches blue on top
      yDiffTop = yBlobs.at(i).yf - bBlobs.at(j).yi; // yellow on top touches blue on bottom 
      uniformTopBott = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).widthEnd); // top width is the same as dx approximately
      uniformTopWhole = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).dx);
      // 
      if (alignsX(yBlobs.at(i),bBlobs.at(j)) ) {
        // std::cout << "Xdiff within threshold" << std::endl;
        if (alignsY(yBlobs.at(i),bBlobs.at(j)) > 0) {
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yf, WO_BEACON_YELLOW_BLUE);
          yb_beacons.push_back(&object);    
        }
        else if (alignsY(yBlobs.at(i),bBlobs.at(j)) < 0) {          
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yi, WO_BEACON_BLUE_YELLOW);
          by_beacons.push_back(&object); 
        }
        if (uniformTopBott < width_thresh) {
          // std::cout << "3 width is uniform from top to bottom" << std::endl;
        }
        if (uniformTopWhole < width_thresh) {
          // std::cout << "4 width is uniform comparing top to whole" << std::endl;
        }
      }
    }
    // std::cout << "Comparing pink now" << std::endl;
    for (int k = 0; k < pBlobs.size(); k++) {
      // std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with pink at [" << pBlobs.at(k).avgX << " ," << pBlobs.at(k).avgY << "]" << std::endl;
      yDiffBottom = pBlobs.at(k).yf - yBlobs.at(i).yi;  // yellow on bottom touches blue on top
      yDiffTop = yBlobs.at(i).yf - pBlobs.at(k).yi; // yellow on top touches blue on bottom 
      uniformTopBott = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).widthEnd); // top width is the same as dx approximately
      uniformTopWhole = std::abs(yBlobs.at(i).widthStart - yBlobs.at(i).dx);
      // 
      if (alignsX(yBlobs.at(i),pBlobs.at(k)) ) {
        // std::cout << "Xdiff within threshold" << std::endl;
        if (alignsY(yBlobs.at(i),pBlobs.at(k) )> 0) {
          // std::cout << "2 yDiffTop within threshold, yellow is just on top of pink" << std::endl;
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yf, WO_BEACON_YELLOW_PINK);
          yp_beacons.push_back(&object); 
        } else if (alignsY(yBlobs.at(i),pBlobs.at(k) )< 0) {
          // std::cout << "1 yDiffBottom within threshold, pink is just on top of yellow" << std::endl;
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yi, WO_BEACON_PINK_YELLOW);   
          py_beacons.push_back(&object);   
        }
        if (uniformTopBott < width_thresh) {
          // std::cout << "3 width is uniform from top to bottom" << std::endl;
        }
        if (uniformTopWhole < width_thresh) {
          // std::cout << "4 width is uniform comparing top to whole" << std::endl;
        }
      }
    }
  }
  for (int p = 0; p < bBlobs.size(); p++) { 
    for (int q = 0; q < pBlobs.size(); q++) {
      // std::cout << "Comparing blue at [" << bBlobs.at(p).avgX << " ," << bBlobs.at(p).avgY << "] with pink at [" << pBlobs.at(q).avgX << " ," << pBlobs.at(q).avgY << "]" << std::endl;
      yDiffBottom = pBlobs.at(q).yf - bBlobs.at(p).yi;  // yellow on bottom touches blue on top
      yDiffTop = bBlobs.at(p).yf - pBlobs.at(q).yi; // yellow on top touches blue on bottom 
      uniformTopBott = std::abs(bBlobs.at(p).widthStart - bBlobs.at(p).widthEnd); // top width is the same as dx approximately
      uniformTopWhole = std::abs(bBlobs.at(p).widthStart - bBlobs.at(p).dx);
      // 
      if (alignsX(bBlobs.at(p),pBlobs.at(q))) {
        // std::cout << "Xdiff within threshold" << std::endl;
        if (alignsY(bBlobs.at(p),pBlobs.at(q)) > 0) {
          // std::cout << "2 yDiffTop within threshold, blue is just on top of pink" << std::endl;
          object = addBeaconObject(bBlobs.at(p).avgX, bBlobs.at(p).yf, WO_BEACON_BLUE_PINK);
          bp_beacons.push_back(&object); 
        } else if (alignsY(bBlobs.at(p),pBlobs.at(q)) < 0) {
          // std::cout << "1 yDiffBottom within threshold, pink is just on top of blue" << std::endl;
          object = addBeaconObject(bBlobs.at(p).avgX, bBlobs.at(p).yi, WO_BEACON_PINK_BLUE);
          pb_beacons.push_back(&object); 
        }
        if (uniformTopBott < width_thresh) {
          // std::cout << "3 width is uniform from top to bottom" << std::endl;
        }
        if (uniformTopWhole < width_thresh) {
          // std::cout << "4 width is uniform comparing top to whole" << std::endl;
        }
      }
    }
  }
}

bool BeaconDetector::alignsX(Blob& blobA, Blob& blobB) {
  int xDiff = std::abs(blobA.avgX - blobB.avgX);
  int sizeDiff = blobA.total - blobB.total;
  float percentDiff = ((float)blobA.total)/((float)blobB.total);
  int aLeftOfBNess = blobB.xi - blobA.xf;
  int bLeftOfANess = blobA.xi - blobB.xf;
  // std::cout << "Difference in X: " << xDiff << " Difference in size(neg means B bigger): " << sizeDiff << " Percent Diff A to B: " << percentDiff << " Leftness of A to B: " << aLeftOfBNess << " Leftness of B to A: " << bLeftOfANess << std::endl;
  if (xDiff < 10) {
    if ((std::abs(1-percentDiff) < 0.75) ) {
     if (aLeftOfBNess < 0 && bLeftOfANess < 0) {
        return true;
      }
    }
  } 
  return false;
}


int BeaconDetector::alignsY(Blob& blobA, Blob& blobB) {
  int yDiff = std::abs(blobA.avgY - blobB.avgY);
  int aTopOfBNess = blobB.yi - blobA.yf;
  int bTopOfANess = blobA.yi - blobB.yf;
  int heightA = blobA.dy;
  int heightB = blobB.dy;
  // std::cout << "Height of A: " << heightA << " Height of B: " << heightB << " Topness of A to B: " << aTopOfBNess << " Topness of B to A: " << bTopOfANess << std::endl;
  
  if (std::abs(aTopOfBNess - bTopOfANess) < (heightA+heightB)*1.5) {
    if (aTopOfBNess > bTopOfANess) {
      // A on Top
      return 1;
    } else {
      // B on Top 
      return -1;
    }
  } 
  return 0;
}

WorldObject BeaconDetector::addBeaconObject(int newCenterX,int newCenterY,WorldObjectType wo_type) {
  WorldObject passObject;
  auto& beaconObject = vblocks_.world_object->objects_[wo_type];
  beaconObject.type = wo_type;
  beaconObject.imageCenterX = newCenterX;
  beaconObject.imageCenterY = newCenterY;
  auto position = cmatrix_.getWorldPosition(beaconObject.imageCenterX, beaconObject.imageCenterY, heights_[wo_type]);
  beaconObject.visionDistance = cmatrix_.groundDistance(position);
  beaconObject.visionBearing = cmatrix_.bearing(position);
  beaconObject.seen = true;
  beaconObject.fromTopCamera = camera_ == Camera::TOP;
  std::cout << "Adding " <<  getName(wo_type) << std::endl;
  tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), beaconObject.imageCenterX, beaconObject.imageCenterY, beaconObject.visionDistance);
  passObject = beaconObject;
  return passObject;
}


void BeaconDetector::chooseBestBeacons(std::vector<std::vector<WorldObject*> > beacon_list) {
  // for (int i = 0; i < beacon_list.size(); i++){
  //   for(int j = 0; j < beacon_list.at(i).size(); j++) {
  //     auto& object = vblocks_.world_object->objects_[beacon_list.at(i).at(j)->type];
  //     object.imageCenterX = beacon_list.at(i).at(j)->imageCenterX;
  //     object.imageCenterY = beacon_list.at(i).at(j)->imageCenterY;
  //     object.visionDistance = beacon_list.at(i).at(j)->visionDistance;
  //     object.visionBearing = beacon_list.at(i).at(j)->visionBearing;
  //     object.seen = true;
  //     object.fromTopCamera = camera_ == Camera::TOP;
  //     std::cout << "Name: " << getName(beacon_list.at(i).at(j)->type) << " CenterX: " << beacon_list.at(i).at(j)->imageCenterX << " CenterY: " <<  beacon_list.at(i).at(j)->imageCenterY << " Vision Distance: " <<  beacon_list.at(i).at(j)->visionDistance << std::endl;
  //     tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(beacon_list.at(i).at(j)->type), object.imageCenterX, object.imageCenterY, object.visionDistance);
  //   }
  // }
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
