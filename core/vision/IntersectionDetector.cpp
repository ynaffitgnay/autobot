#include <vision/IntersectionDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

IntersectionDetector::IntersectionDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void IntersectionDetector::findIntersections(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  // printf("\n\nNew frame\n");
  // std::cout << std::endl;
  // std::cout << "Frame ID: " << vblocks_.frame_info->frame_id << std::endl;
  // std::cout << std::endl;
  std::vector<Blob> blueCand;
  std::vector<Blob> pinkCand;
  std::vector<Blob> yellowCand;
  std::vector<WorldObject> y_intersections;
  std::vector<WorldObject> p_intersections;
  std::vector<std::vector<WorldObject> > intersection_list;
  for (int i = 0; i < blobs.size(); i++) {
  switch (blobs.at(i).color)
      {  
        case c_BLUE : 
          // std::cout << "Pushed back blue" << std::endl;
          if (blobs.at(i).total > 10){
            blueCand.push_back(blobs.at(i));
          }
          break;
        case c_YELLOW :
          // std::cout << "Pushed back yellow" << std::endl;
          if (blobs.at(i).total > 10){
            yellowCand.push_back(blobs.at(i));
          }
          break;
        case c_PINK :
          // std::cout << "Pushed back pink" << std::endl;  
          if (blobs.at(i).total > 10){
            pinkCand.push_back(blobs.at(i));
          }
          break;
        default: // None of the intersection colors
          break;       
    }
  }

  processBlobs(yellowCand,pinkCand,blueCand,y_intersections, p_intersections);
  
  chooseBestIntersections(y_intersections,p_intersections);
}

void IntersectionDetector::processBlobs(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs
                                      , std::vector<WorldObject>& y_intersections, std::vector<WorldObject>& p_intersections ) {
  WorldObject object;
  bool mightBeABeacon = false;

  // Going through yellow blobs
  for (int i = 0; i < yBlobs.size(); i++) {
    mightBeABeacon = false;
    if (bBlobs.size() > 0) {
      // Going through blue compared with yellow
      for (int j = 0; j < bBlobs.size(); j++) {
        // std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with blue at [" << bBlobs.at(j).avgX << " ," << bBlobs.at(j).avgY << "]" << std::endl;
        if (neighbors(yBlobs.at(i),bBlobs.at(j))) {
          mightBeABeacon = true;
        } 
      }
    } 
    if (pBlobs.size() > 0) {
      for (int k = 0; k < pBlobs.size(); k++) {
        // std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with pink at [" << pBlobs.at(k).avgX << " ," << pBlobs.at(k).avgY << "]" << std::endl;
        if (neighbors(yBlobs.at(i),pBlobs.at(k))) {
          mightBeABeacon = true;
        } 
      }
    } 
    if (yBlobs.at(i).correctPixelRatio < 1.8) {
      // printf("Yellow pixel ratio: %f\n",yBlobs.at(i).correctPixelRatio);
      mightBeABeacon = true;
    }
    if (!mightBeABeacon) {
      // printf("Yellow intersection of size:%d at [%d, %d]\n",yBlobs.at(i).total, yBlobs.at(i).avgX, yBlobs.at(i).avgY);
      object = addIntersectionObject(yBlobs.at(i).avgX,yBlobs.at(i).avgY, yBlobs.at(i).correctPixelRatio);
      y_intersections.push_back(object);
    }
  }

  // Going through pink blobs now
  for (int p = 0; p < pBlobs.size(); p++) { 
    mightBeABeacon = false;
    if (bBlobs.size() > 0) {
      // Going through blue compared with yellow
      for (int q = 0; q < bBlobs.size(); q++) {
        // std::cout << "Comparing pink at [" << pBlobs.at(p).avgX << " ," << pBlobs.at(p).avgY << "] with blue at [" << bBlobs.at(q).avgX << " ," << bBlobs.at(q).avgY << "]" << std::endl;
        if (neighbors(pBlobs.at(p),bBlobs.at(q)) ) {
          mightBeABeacon = true;
        } 
      }
    }
    if (yBlobs.size() > 0) {
      for (int r = 0; r < yBlobs.size(); r++) {
        // std::cout << "Comparing pink at [" << pBlobs.at(p).avgX << " ," << pBlobs.at(p).avgY << "] with yellow at [" << yBlobs.at(r).avgX << " ," << yBlobs.at(r).avgY << "] " << std::endl;
        if (neighbors(pBlobs.at(p),yBlobs.at(r))) {
          mightBeABeacon = true;
        }
      }
    } 
    if (pBlobs.at(p).correctPixelRatio < 1.8) {
      // printf("Pink pixel ratio: %f\n",pBlobs.at(p).correctPixelRatio);
      mightBeABeacon = true;
    }
    if (!mightBeABeacon) {
      // printf("Pink intersection of size: %d at [%d, %d]\n", pBlobs.at(p).total, pBlobs.at(p).avgX, pBlobs.at(p).avgY);
      object = addIntersectionObject(pBlobs.at(p).avgX, pBlobs.at(p).avgY, pBlobs.at(p).correctPixelRatio);
      p_intersections.push_back(object);
    }
  }
}

bool IntersectionDetector::neighbors(Blob& blobA, Blob& blobB) {
  int minDist1X = std::min(std::abs(blobA.xi - blobB.xi), std::abs(blobA.xf - blobB.xi));
  int minDist2X = std::min(std::abs(blobA.xi - blobB.xf), std::abs(blobA.xf - blobB.xf));
  int minDistX = std::min(minDist1X,minDist2X);
  int minDistY = std::min(std::abs(blobA.yi - blobB.yf), std::abs(blobA.yf - blobB.yi));
  if (minDistX > 10 || minDistY > 10) {  // X pixels are far apart or the y pixels are far apart: not a neighbor
    // printf("Min dist between blobA and blobB is %d, %d\n", minDistX, minDistY);

    return false;
  } 
  // probably a neighbor and thus might be a beacon
  return true;
}


WorldObject IntersectionDetector::addIntersectionObject(int newCenterX,int newCenterY,float aspectRatio) {
  WorldObject intersectionObject;
  // auto& intersectionObject = vblocks_.world_object->objects_[wo_type];
  // intersectionObject.type = wo_type;

  intersectionObject.imageCenterX = newCenterX;
  intersectionObject.imageCenterY = newCenterY;
  // printf("Old height: %d New height: %d\n", heights_[wo_type], newHeight);
  auto position = cmatrix_.getWorldPosition(intersectionObject.imageCenterX, intersectionObject.imageCenterY, 0);
  intersectionObject.visionDistance = cmatrix_.groundDistance(position);
  intersectionObject.visionBearing = cmatrix_.bearing(position);
  intersectionObject.seen = true;
  intersectionObject.fromTopCamera = camera_ == Camera::TOP;
  // tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), intersectionObject.imageCenterX, intersectionObject.imageCenterY, intersectionObject.visionDistance);
  // passObject = intersectionObject;
  return intersectionObject;
}




void IntersectionDetector::chooseBestIntersections(std::vector<WorldObject>& y_list, std::vector<WorldObject>& p_list) {
  // Cycle through beacon types
  // for (int i = 0; i < y_list.size(); i++){
  //   printf("Width yellow: %d\n",y_list.at(i).width);
  // }

  // for (int j = 0; j < p_list.size(); j++){
  //   printf("Width pink: %d\n",p_list.at(j).width);
  // }

}

