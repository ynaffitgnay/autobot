#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BeaconDetector::findBeacons(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  std::vector<Blob> blueCand;
  std::vector<Blob> pinkCand;
  std::vector<Blob> yellowCand;
  std::vector<Blob> whiteCand;
  std::vector<WorldObject> yb_beacons;
  std::vector<WorldObject> by_beacons;
  std::vector<WorldObject> yp_beacons;
  std::vector<WorldObject> py_beacons;
  std::vector<WorldObject> bp_beacons;
  std::vector<WorldObject> pb_beacons;
  std::vector<std::vector<WorldObject> > beacon_list;
  for (int i = 0; i < blobs.size(); i++) {
    // auto position = cmatrix_.getWorldPosition(blobs.at(i).avgX, blobs.at(i).avgY, 150); 
    // auto visionDistance = cmatrix_.groundDistance(position);
    // auto visionBearing = cmatrix_.bearing(position);
    // std::cout << "Saw " << i << " at [" << blobs.at(i).avgX << " ," << blobs.at(i).avgY << "] with calculated distance " << visionDistance << std::endl;
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
      case c_WHITE:
        whiteCand.push_back(blobs.at(i));
        break;
      default: // None of the beacon colors
        break;       
    }
  }

  genCombos(yellowCand,pinkCand,blueCand,whiteCand,yb_beacons,by_beacons,yp_beacons,py_beacons,bp_beacons,pb_beacons);
  // std::cout << "YB beacons: " << yb_beacons.size() << " BY beacons: " << by_beacons.size() << " YP beacons: " << yp_beacons.size() << " PY beacons: " << py_beacons.size() << " BP beacons: " << bp_beacons.size() << " PB beacons: " << pb_beacons.size() << std::endl;
  
  beacon_list.push_back(yb_beacons);
  beacon_list.push_back(by_beacons);
  beacon_list.push_back(yp_beacons);
  beacon_list.push_back(py_beacons);
  beacon_list.push_back(bp_beacons);
  beacon_list.push_back(pb_beacons);
  chooseBestBeacons(beacon_list);
}

void BeaconDetector::genCombos(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs, std::vector<Blob>& wBlobs,
                               std::vector<WorldObject>& yb_beacons, std::vector<WorldObject>& by_beacons,
                               std::vector<WorldObject>& yp_beacons, std::vector<WorldObject>& py_beacons,
                               std::vector<WorldObject>& bp_beacons, std::vector<WorldObject>& pb_beacons ) {
  int height_thresh = 4;
  int width_thresh = 4;
  int xDiff;
  int yDiffBottom;
  int yDiffTop;
  int uniformTopBott;
  int uniformTopWhole;
  WorldObject object;

  // Going through yellow blobs
  for (int i = 0; i < yBlobs.size(); i++) {

    // Going through blue compared with yellow
    for (int j = 0; j < bBlobs.size(); j++) {
      // std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with blue at [" << bBlobs.at(j).avgX << " ," << bBlobs.at(j).avgY << "]" << std::endl;
      if (alignsX(yBlobs.at(i),bBlobs.at(j)) ) {
        // std::cout << "Xdiff within threshold" << std::endl;
        if (alignsY(yBlobs.at(i),bBlobs.at(j)) > 0) {
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yf, WO_BEACON_YELLOW_BLUE);
          if(objectValidInWorld(object,yBlobs.at(i),bBlobs.at(j)))  yb_beacons.push_back(object);    
        }
        else if (alignsY(yBlobs.at(i),bBlobs.at(j)) < 0) {          
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yi, WO_BEACON_BLUE_YELLOW);
          if(objectValidInWorld(object,bBlobs.at(j),yBlobs.at(i)))  by_beacons.push_back(object); 
        }
      }
    }

    // Going through pink compared with yellow
    // std::cout << "Comparing pink now" << std::endl;
    for (int k = 0; k < pBlobs.size(); k++) {
      // std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with pink at [" << pBlobs.at(k).avgX << " ," << pBlobs.at(k).avgY << "]" << std::endl;
     if (alignsX(yBlobs.at(i),pBlobs.at(k)) ) {
        // std::cout << "Xdiff within threshold" << std::endl;
        if (alignsY(yBlobs.at(i),pBlobs.at(k) )> 0) {
          // std::cout << "2 yDiffTop within threshold, yellow is just on top of pink" << std::endl;
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yf, WO_BEACON_YELLOW_PINK);
          if(objectValidInWorld(object,yBlobs.at(i),pBlobs.at(k)))  yp_beacons.push_back(object); 
        } else if (alignsY(yBlobs.at(i),pBlobs.at(k) )< 0) {
          // std::cout << "1 yDiffBottom within threshold, pink is just on top of yellow" << std::endl;
          object = addBeaconObject(yBlobs.at(i).avgX, yBlobs.at(i).yi, WO_BEACON_PINK_YELLOW);   
          if(objectValidInWorld(object,pBlobs.at(k),yBlobs.at(i)))  py_beacons.push_back(object);   
        }
      }
    }
  } // Outside of yellow now

  // Going through blue blobs again
  for (int p = 0; p < bBlobs.size(); p++) { 

  // Going through blue compared with pink
    for (int q = 0; q < pBlobs.size(); q++) {
      // std::cout << "Comparing blue at [" << bBlobs.at(p).avgX << " ," << bBlobs.at(p).avgY << "] with pink at [" << pBlobs.at(q).avgX << " ," << pBlobs.at(q).avgY << "]" << std::endl;
      if (alignsX(bBlobs.at(p),pBlobs.at(q))) {
        // std::cout << "Xdiff within threshold" << std::endl;
        if (alignsY(bBlobs.at(p),pBlobs.at(q)) > 0) {
          // std::cout << "2 yDiffTop within threshold, blue is just on top of pink" << std::endl;
          object = addBeaconObject(bBlobs.at(p).avgX, bBlobs.at(p).yf, WO_BEACON_BLUE_PINK);
          if(objectValidInWorld(object,bBlobs.at(p),pBlobs.at(q)))  bp_beacons.push_back(object); 
        } else if (alignsY(bBlobs.at(p),pBlobs.at(q)) < 0) {
          // std::cout << "1 yDiffBottom within threshold, pink is just on top of blue" << std::endl;
          object = addBeaconObject(bBlobs.at(p).avgX, bBlobs.at(p).yi, WO_BEACON_PINK_BLUE);
          if(objectValidInWorld(object,pBlobs.at(q),bBlobs.at(p)))  pb_beacons.push_back(object); 
        }
      }
    }
  }
}

bool BeaconDetector::alignsX(Blob& blobA, Blob& blobB) {
  int xDiff = std::abs(blobA.avgX - blobB.avgX);  // Difference in center x values
  float ratioBlobs = ((float)blobA.total)/((float)blobB.total); // Ratio of blob sizes
  int aLeftOfBNess = blobB.xi - blobA.xf;  // A measure of how left a is compared to b. (It's negative if it's not left)
  int bLeftOfANess = blobA.xi - blobB.xf;  // A measure of how left b is compared to a. (Same)
  float blobDensityA = blobA.pixelDensity;
  float blobDensityB = blobB.pixelDensity;
  // std::cout << "Difference in X: " << xDiff << " Difference in size(neg means B bigger): " << sizeDiff << " Percent Diff A to B: " << ratioBlobs << std::endl;
  // std::cout << "Leftness of A to B: " << aLeftOfBNess << " Leftness of B to A: " << bLeftOfANess << " Pixel Density A: " << blobDensityA << " Pixel Density B" << blobDensityB << std::endl;
  
  // Checking fitness conditions
  if (xDiff < 10) {  // Some hardcoded values. Sorry
    if ((std::abs(1-ratioBlobs) < 0.75) ) {
     if (aLeftOfBNess < 0 && bLeftOfANess < 0) {
        if (blobDensityB > 0.3 && blobDensityA > 0.28) {
          return true;
        }
      }
    }
  } 
  return false;
}


int BeaconDetector::alignsY(Blob& blobA, Blob& blobB) {
  int yDiff = std::abs(blobA.avgY - blobB.avgY);   // Difference in center y values
  int aTopOfBNess = blobB.yi - blobA.yf;   // A measure of how far above a is compared to b. (It's negative if it's not on top)
  int bTopOfANess = blobA.yi - blobB.yf;   // ^^ Same but reversed
  int heightA = blobA.dy;
  int heightB = blobB.dy;
  // std::cout << "Height of A: " << heightA << " Height of B: " << heightB << " Topness of A to B: " << aTopOfBNess << " Topness of B to A: " << bTopOfANess << std::endl;
  
  // Checking fitness conditions
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
  WorldObject beaconObject;
  // auto& beaconObject = vblocks_.world_object->objects_[wo_type];
  beaconObject.type = wo_type;
  beaconObject.imageCenterX = newCenterX;
  beaconObject.imageCenterY = newCenterY;
  auto position = cmatrix_.getWorldPosition(beaconObject.imageCenterX, beaconObject.imageCenterY, heights_[wo_type]);
  beaconObject.visionDistance = cmatrix_.groundDistance(position);
  beaconObject.visionBearing = cmatrix_.bearing(position);
  beaconObject.seen = true;
  beaconObject.fromTopCamera = camera_ == Camera::TOP;
  tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), beaconObject.imageCenterX, beaconObject.imageCenterY, beaconObject.visionDistance);
  // passObject = beaconObject;
  return beaconObject;
}

bool BeaconDetector::objectValidInWorld(WorldObject& object, Blob& topBlob, Blob& bottomBlob) {
  float width = cmatrix_.getCameraWidthByDistance(object.visionDistance, 110);
  float height = cmatrix_.getCameraHeightByDistance(object.visionDistance, 100);
  float topratioWidth = width/((float)topBlob.xf-topBlob.xi);
  float topratioHeight = height/((float)topBlob.yf-topBlob.yi);
  float bottomratioWidth = width/((float)bottomBlob.xf-bottomBlob.xi);
  float bottomratioHeight = height/((float)bottomBlob.yf-bottomBlob.yi);
  float ratioThresh;

  // Scaling the threshold of what the ratio can look like based on the size of the whole blob. 
  // more of a difference on small blobs
  if (width*height - topBlob.total > 600) {
    ratioThresh = 0.4;
  }
  else{
    ratioThresh = 1.5;
  }
  // printf("Width = %f, Height = %f, Total (est): %f Object Type = %s\n", width, height, width*height, getName(object.type));
  // printf("W_topBlob = %d, H_topBlob = %d, total top: %d ratio_topWidth = %f, ratio_topHeight = %f\n", topBlob.xf - topBlob.xi, topBlob.yf - topBlob.yi, topBlob.total, topratioWidth, topratioHeight);
  // printf("W_bottomBlob = %d, H_bottomBlob = %d, ratio_bottomWidth = %f, ratio_bottomHeight = %f\n",bottomBlob.xf - bottomBlob.xi, bottomBlob.yf - bottomBlob.yi, bottomratioWidth, bottomratioHeight);
  

  // We want the width the camera believes the beacon is to be comparable with what the pixels tell us the width is
  if (((std::abs(1-topratioWidth) < ratioThresh)+ (std::abs(1-topratioHeight) < ratioThresh) + (std::abs(1-bottomratioWidth) < ratioThresh) + (std::abs(1-bottomratioHeight) < ratioThresh)) >= 3) 
  {
    // if (object.visionDistance > 4000){
    //   return false;
    // }

    // We'll use this confidence later when comparing possible beacons to try to use the best fit. This handles the upside down beacons very well.
    object.visionConfidence = std::abs(1-topratioWidth)+ std::abs(1-topratioWidth) + std::abs(1-topratioWidth) + std::abs(1-topratioWidth);
    return true;
  }
  return false;
}


void BeaconDetector::chooseBestBeacons(std::vector<std::vector<WorldObject> >& beacon_list) {

  // Cycle through beacon types
  for (int i = 0; i < beacon_list.size(); i++){

    // A lot of setup
    WorldObject bestBeacon;
    WorldObject evalBeacon;
    bool rejected = false;
    float bestFitness = 99.0;

    // Cycle through each beacon in the current type
    for(int j = 0; j < beacon_list.at(i).size(); j++) {
      
        evalBeacon = beacon_list.at(i).at(j);

        // Cycle through types again
        for (int c = 0; c < beacon_list.size(); c++){
          WorldObject neighborBeacon;

          // Cycle through each beacon again
          for(int d = 0; d < beacon_list.at(c).size(); d++) {

            // do not compare against self
            if (i != c && j != d) {
              neighborBeacon = beacon_list.at(c).at(d);
              float heightEvalBeacon = cmatrix_.getCameraHeightByDistance(evalBeacon.visionDistance, 100);
              float heightNeighborBeacon = cmatrix_.getCameraHeightByDistance(neighborBeacon.visionDistance, 100);
              float widthEvalBeacon = cmatrix_.getCameraWidthByDistance(evalBeacon.visionDistance, 110);
              float widthNeighborBeacon = cmatrix_.getCameraWidthByDistance(neighborBeacon.visionDistance, 110);
              float centerEvalX = evalBeacon.imageCenterX;
              float centerNeighborX = neighborBeacon.imageCenterX;
              float centerEvalY = evalBeacon.imageCenterY;
              float centerNeighborY = neighborBeacon.imageCenterY;
              // std::cout << "Type Eval: " << getName(evalBeacon.type) << " Type Neighbor: " << getName(neighborBeacon.type) <<" Height Eval: " << heightEvalBeacon << " Height Neighbor: " << heightNeighborBeacon << std::endl;
              // std::cout << "CenterX Eval: " << centerEvalX << " CenterX Neighbor: " << centerNeighborX << " CenterY Eval: " << centerEvalY << " CenterY Neighbor: " << centerNeighborY << std::endl;
              
              // Comparing to see if there is another beacon that is vertically aligned with our current right on top or right below it. If so, we reject it.
              if (std::abs(centerEvalX - centerNeighborX) < widthEvalBeacon) {
                if ( (std::abs(centerEvalY-centerNeighborY)-(heightEvalBeacon+heightNeighborBeacon)) < std::max(heightEvalBeacon,heightNeighborBeacon) ){
                  // std::cout << "Reject Beacon" << std::endl;
                  rejected = true;
                } 
              }
            }
          }
        }

        // Checking that fitness criteria from objectValidInWorld
        if (beacon_list.at(i).at(j).visionConfidence < bestFitness && rejected == false) {
          bestBeacon = beacon_list.at(i).at(j);
          bestFitness = bestBeacon.visionConfidence;
          rejected = false;
        }
    }

    // Add only the best beacon of each type to the actual world objects list
    if (beacon_list.at(i).size() > 0 && rejected == false ) {
      auto& object = vblocks_.world_object->objects_[bestBeacon.type];
      object.imageCenterX = bestBeacon.imageCenterX;
      object.imageCenterY = bestBeacon.imageCenterY;
      object.visionDistance = bestBeacon.visionDistance;
      object.visionBearing = bestBeacon.visionBearing;
      object.seen = true;
      object.fromTopCamera = camera_ == Camera::TOP;
      // std::cout << "Adding: " << getName(bestBeacon.type) << " CenterX: " << bestBeacon.imageCenterX << " CenterY: " <<  bestBeacon.imageCenterY << " Vision Distance: " <<  bestBeacon.visionDistance << std::endl;
      tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(bestBeacon.type), object.imageCenterX, object.imageCenterY, object.visionDistance);
    }
  }
}

