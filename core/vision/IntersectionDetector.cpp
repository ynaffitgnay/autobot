#include <vision/IntersectionDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

IntersectionDetector::IntersectionDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void IntersectionDetector::findIntersections(std::vector<Blob>& blobs) {
  // printf("\n\nNew frame\n");
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
    // printf("Yellow pixel ratio: %f Xdiff: %d Ydiff: %d Calc ratio: %f\n",yBlobs.at(i).correctPixelRatio, yBlobs.at(i).dx, yBlobs.at(i).dy, (float)yBlobs.at(i).dx/ (float)yBlobs.at(i).dy);
    if (yBlobs.at(i).correctPixelRatio < 1.8) {
      mightBeABeacon = true;
    }
    if (!mightBeABeacon) {
      // printf("Yellow intersection of size:%d at [%d, %d]\n",yBlobs.at(i).total, yBlobs.at(i).avgX, yBlobs.at(i).avgY);
      int xdiff = yBlobs.at(i).dx;
      int ydiff = yBlobs.at(i).dy;
      // printf("Xdiff: %d Ydiff: %d\n", xdiff,ydiff);
      object = addIntersectionObject(yBlobs.at(i).avgX,yBlobs.at(i).avgY, xdiff, ydiff);
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
    // printf("Pink pixel ratio: %f Xdiff: %d Ydiff: %d Calc ratio: %f\n",pBlobs.at(p).correctPixelRatio, pBlobs.at(p).dx, pBlobs.at(p).dy, (float)pBlobs.at(p).dx/ (float)pBlobs.at(p).dy);
    if (pBlobs.at(p).correctPixelRatio < 1.8) {
      mightBeABeacon = true;
    }
    if (!mightBeABeacon) {
      // printf("Pink intersection of size: %d at [%d, %d]\n", pBlobs.at(p).total, pBlobs.at(p).avgX, pBlobs.at(p).avgY);
      int xdiff = pBlobs.at(p).dx;
      int ydiff = pBlobs.at(p).dy;
      // printf("Xdiff: %d Ydiff: %d\n", xdiff,ydiff);
      object = addIntersectionObject(pBlobs.at(p).avgX, pBlobs.at(p).avgY, xdiff, ydiff);
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


WorldObject IntersectionDetector::addIntersectionObject(int newCenterX,int newCenterY,int xdiff, int ydiff) {
  WorldObject intersectionObject;
  // auto& intersectionObject = vblocks_.world_object->objects_[wo_type];
  // intersectionObject.type = wo_type;

  intersectionObject.imageCenterX = newCenterX;
  intersectionObject.imageCenterY = newCenterY;
  auto position = cmatrix_.getWorldPosition(intersectionObject.imageCenterX, intersectionObject.imageCenterY, 0);
  intersectionObject.visionDistance = cmatrix_.groundDistance(position);
  intersectionObject.visionBearing = cmatrix_.bearing(position);
  

  int lineWidth;
  int half_width;
  if (xdiff > 2 * ydiff) {
    int xf_new = newCenterX + std::round(xdiff*0.5);
    int xi_new = newCenterX - std::round(xdiff*0.5);

    auto i_pos = cmatrix_.getWorldPosition(xi_new,newCenterY,0);
    auto f_pos = cmatrix_.getWorldPosition(xf_new,newCenterY,0);
    float test_width = i_pos.y - f_pos.y;
    // printf("Init: [%f, %f] Final: [%f, %f] Center: [%f, %f] test_width: %f\n", i_pos.x, i_pos.y, f_pos.x, f_pos.y, position.x, position.y,test_width);
    lineWidth = test_width;
  } else if (ydiff > 2*xdiff) {
    int yf_new = newCenterY - std::round(ydiff*0.5);
    int yi_new = newCenterY + std::round(ydiff*0.5);

    auto i_pos = cmatrix_.getWorldPosition(newCenterX, yi_new,0);
    auto f_pos = cmatrix_.getWorldPosition(newCenterX, yf_new,0);
    float test_width = i_pos.y - f_pos.y;
    // printf("Init: [%f, %f] Final: [%f, %f] Center: [%f, %f] test_width: %f\n", i_pos.x, i_pos.y, f_pos.x, f_pos.y, position.x, position.y,test_width);
    lineWidth =  test_width;
  } else {
    lineWidth = 0;
  }


  intersectionObject.width = lineWidth;
  intersectionObject.seen = true;
  intersectionObject.fromTopCamera = camera_ == Camera::TOP;
  // tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(wo_type), intersectionObject.imageCenterX, intersectionObject.imageCenterY, intersectionObject.visionDistance);
  // passObject = intersectionObject;
  return intersectionObject;
}




void IntersectionDetector::chooseBestIntersections(std::vector<WorldObject>& y_list, std::vector<WorldObject>& p_list) {
  // Cycle through beacon types
    // If there is only one intersection seen in the top camera (assuming we always face the center)
  bool named_p = false;
  bool named_y = false;
  if (y_list.size() == 0) named_y = true;
  if (p_list.size() == 0) named_p = true;
  if (named_y && named_p) return;

  // if we see more than one we have to be near the center on the other side of it looking at the opp goal side
  int max_width = 0;
  int max_index = 0;
  if (!named_y) {
    if (y_list.size() > 1) {  // can see both intersections of one color
      // printf("Dist yellow1: %f Dist yellow 2: %f\n",y_list.at(0).visionDistance, y_list.at(1).visionDistance);
      if (y_list.at(0).visionDistance > y_list.at(1).visionDistance) {
        y_list.at(0).type = WO_OPP_PEN_RIGHT_T;
        y_list.at(1).type = WO_OPP_PEN_RIGHT_L;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(y_list.at(0).type), y_list.at(0).imageCenterX, y_list.at(0).imageCenterY, y_list.at(0).visionDistance, y_list.at(0).visionBearing*180.0/M_PI);
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(y_list.at(1).type), y_list.at(1).imageCenterX, y_list.at(1).imageCenterY, y_list.at(1).visionDistance, y_list.at(1).visionBearing*180.0/M_PI);
      } else {
        y_list.at(0).type = WO_OPP_PEN_RIGHT_L;
        y_list.at(1).type = WO_OPP_PEN_RIGHT_T;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(y_list.at(0).type), y_list.at(0).imageCenterX, y_list.at(0).imageCenterY, y_list.at(0).visionDistance, y_list.at(0).visionBearing*180.0/M_PI);
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(y_list.at(1).type), y_list.at(1).imageCenterX, y_list.at(1).imageCenterY, y_list.at(1).visionDistance, y_list.at(1).visionBearing*180.0/M_PI);
      }
    } else {  // only one intersection
      // printf("Width yellow: %f at [%d, %d] with distance %f and bearing %f and adjusted width %f\n",y_list.at(0).width, y_list.at(0).imageCenterX, y_list.at(0).imageCenterY, y_list.at(0).visionDistance, y_list.at(0).visionBearing*180.0/M_PI, y_list.at(0).width);
      float case1 = std::abs(1.0 - y_list.at(0).width/375.0);
      float case2 = std::abs(1.0 - y_list.at(0).width/225.0);
      // printf("T case: %f L case: %f\n", case1, case2);
      if (case1 < case2) {
        y_list.at(0).type = WO_OWN_PEN_LEFT_T;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(y_list.at(0).type), y_list.at(0).imageCenterX, y_list.at(0).imageCenterY, y_list.at(0).visionDistance, y_list.at(0).visionBearing*180.0/M_PI);
      } else {
        y_list.at(0).type = WO_OPP_PEN_RIGHT_L;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(y_list.at(0).type), y_list.at(0).imageCenterX, y_list.at(0).imageCenterY, y_list.at(0).visionDistance, y_list.at(0).visionBearing*180.0/M_PI);
      }   
    }
  }

  if (!named_p) {
    if (p_list.size() > 1) {  // can see both intersections of one color
      // printf("Dist pink1: %f Dist pink 2: %f\n",p_list.at(0).visionDistance, p_list.at(1).visionDistance);
      if (p_list.at(0).visionDistance > p_list.at(1).visionDistance) {
        p_list.at(0).type = WO_OPP_PEN_LEFT_T;
        p_list.at(1).type = WO_OPP_PEN_LEFT_L;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(p_list.at(0).type), p_list.at(0).imageCenterX, p_list.at(0).imageCenterY, p_list.at(0).visionDistance, p_list.at(0).visionBearing*180.0/M_PI);
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(p_list.at(1).type), p_list.at(1).imageCenterX, p_list.at(1).imageCenterY, p_list.at(1).visionDistance, p_list.at(1).visionBearing*180.0/M_PI);
      } else {
        p_list.at(0).type = WO_OPP_PEN_LEFT_L;
        p_list.at(1).type = WO_OPP_PEN_LEFT_T;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(p_list.at(0).type), p_list.at(0).imageCenterX, p_list.at(0).imageCenterY, p_list.at(0).visionDistance, p_list.at(0).visionBearing*180.0/M_PI);
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(p_list.at(1).type), p_list.at(1).imageCenterX, p_list.at(1).imageCenterY, p_list.at(1).visionDistance, p_list.at(1).visionBearing*180.0/M_PI);
      }
    } else {  // only one intersection
      // printf("Width pink: %f at [%d, %d] with distance %f and bearing %f and adjusted width %f\n",p_list.at(0).width, p_list.at(0).imageCenterX, p_list.at(0).imageCenterY, p_list.at(0).visionDistance, p_list.at(0).visionBearing*180.0/M_PI);
      float case1 = std::abs(1.0 - p_list.at(0).width/375.0);
      float case2 = std::abs(1.0 - p_list.at(0).width/225.0);
      // printf("T case: %f L case: %f\n", case1, case2);
      if (case1 < case2) {
        p_list.at(0).type = WO_OWN_PEN_RIGHT_T;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(p_list.at(0).type), p_list.at(0).imageCenterX, p_list.at(0).imageCenterY, p_list.at(0).visionDistance, p_list.at(0).visionBearing*180.0/M_PI);
      } else {
        p_list.at(0).type = WO_OPP_PEN_LEFT_L;
        // printf("Saw %s at [%d, %d] with distance %f and bearing %f\n", getName(p_list.at(0).type), p_list.at(0).imageCenterX, p_list.at(0).imageCenterY, p_list.at(0).visionDistance, p_list.at(0).visionBearing*180.0/M_PI);
      }   
    }
  }

}