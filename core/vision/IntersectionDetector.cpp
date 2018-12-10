#include <vision/IntersectionDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

IntersectionDetector::IntersectionDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void IntersectionDetector::findIntersections(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
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
        default: // None of the intersection colors
          break;       
    }
  }

  processBlobs(yellowCand,pinkCand,blueCand,y_intersections, p_intersections);
  
  intersection_list.push_back(y_intersections);
  intersection_list.push_back(p_intersections);
  chooseBestIntersections(intersection_list);
}

void IntersectionDetector::processBlobs(std::vector<Blob>& yBlobs, std::vector<Blob>& pBlobs, std::vector<Blob>& bBlobs
                                      , std::vector<WorldObject>& y_intersections, std::vector<WorldObject>& p_intersections ) {
  WorldObject object;
  bool mightBeAIntersection = false;

  // Going through yellow blobs
  for (int i = 0; i < yBlobs.size(); i++) {
    mightBeAIntersection = false;
    if (bBlobs.size() > 0) {
      // Going through blue compared with yellow
      for (int j = 0; j < bBlobs.size(); j++) {
        std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with blue at [" << bBlobs.at(j).avgX << " ," << bBlobs.at(j).avgY << "]" << std::endl;
        if (neighbors(yBlobs.at(i),bBlobs.at(j))) {
          mightBeAIntersection = true;
        }
      }
    } else if (pBlobs.size() > 0) {
      for (int k = 0; k < pBlobs.size(); k++) {
        std::cout << "Comparing yellow at [" << yBlobs.at(i).avgX << " ," << yBlobs.at(i).avgY << "] with pink at [" << pBlobs.at(k).avgX << " ," << pBlobs.at(k).avgY << "]" << std::endl;
        if (neighbors(yBlobs.at(i),pBlobs.at(k))) {
          mightBeAIntersection = true;
        }
      }
    } 
    if (!mightBeAIntersection) {
      object = addIntersectionObject(yBlobs.at(i).avgX,yBlobs.at(i).avgY);
      if(objectValidInWorld(object,yBlobs.at(i))) {
         y_intersections.push_back(object);
      }     
    }
  }

  // Going through pink blobs now
  for (int p = 0; p < pBlobs.size(); p++) { 
    mightBeAIntersection = false;
    if (bBlobs.size() > 0) {
      // Going through blue compared with yellow
      for (int q = 0; q < bBlobs.size(); q++) {
        std::cout << "Comparing pink at [" << pBlobs.at(p).avgX << " ," << pBlobs.at(p).avgY << "] with blue at [" << bBlobs.at(q).avgX << " ," << bBlobs.at(q).avgY << "]" << std::endl;
        if (neighbors(pBlobs.at(p),bBlobs.at(q)) ) {
          mightBeAIntersection = true;
        }
      }
    } 
    if (!mightBeAIntersection) {
      object = addIntersectionObject(pBlobs.at(p).avgX, pBlobs.at(p).avgY);
      if(objectValidInWorld(object,pBlobs.at(p))) {
        p_intersections.push_back(object);
      }     
    }
  }
}

bool IntersectionDetector::neighbors(Blob& blobA, Blob& blobB) {
  int minDist1 = std::min(std::abs(blobA.xi - blobB.xi), std::abs(blobA.xf - blobB.xi));
  int minDist2 = std::min(std::abs(blobA.xi - blobB.xf), std::abs(blobA.xf - blobB.xf));
  int minDistOverall = std::min(minDist1,minDist2);
  printf("Min dist between blobA and blobB is %d", minDistOverall);
  if (minDistOverall > 10) {
    return false;
  } 
  return true;
}


WorldObject IntersectionDetector::addIntersectionObject(int newCenterX,int newCenterY) {
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

bool IntersectionDetector::objectValidInWorld(WorldObject& object, Blob& blob) {
  return true;
}


void IntersectionDetector::chooseBestIntersections(std::vector<std::vector<WorldObject> >& intersection_list) {


}

