#include <vision/BallDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>
#include <vision/ImageProcessor.h>
#include <vision/structures/Blob.h>
#include <common/ColorConversion.h>
#include <iostream>
#include <cmath>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace Eigen;

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

unsigned char* BallDetector::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* BallDetector::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

void BallDetector::findBall(std::vector<Blob>& blobs, std::vector<BallCandidate*>& ballCands) {
  // int ballHeightMM = 33;
  
  auto fid = vblocks_.frame_info->frame_id;
  
  bool ballDetected = false;
  Blob* orangeBlob = nullptr;
  BallCandidate* newCand = nullptr;
  int i = 0;
  for (auto blob : blobs) {
    if (blob.color == c_ORANGE) { //maybe update pixel ratio here too!!
      orangeBlob = &blob;

      // Now some heuristics
      if (orangeBlob->total < 8) {
        //std::cout << "Eliminated for being too small.\n";
        break;
      }

      if (camera_ == Camera::TOP) {
        if (orangeBlob->total > 900) {
          //std::cout << "Eliminated for being too large.\n";
          continue;
        }
      }
      
      float ratioHighFactor = 1.45;
      float ratioLowFactor = 0.65;
      
      // Check that the ratio makes sense
      if (orangeBlob->correctPixelRatio != 0 &&
          (orangeBlob->correctPixelRatio > ratioHighFactor ||
           orangeBlob->correctPixelRatio < ratioLowFactor)) {
        //std::cout << "Eliminated for pixel ratio\n";
        continue;
      }

      if (!checkBottomColor(orangeBlob)) {
        continue;
      }
      
      
      // Make sure that the center of the object (check a couple of points around the middle) are orange
      // (rule out the white folder with orange)
      if (!checkCenter(orangeBlob)) {
        //std::cout << "Eliminated for non-orange center\n";
        continue;
      }
      
      // Make sure no other blob within (factor) of size is located at the same position

      
      newCand = new BallCandidate;
      
      newCand->centerX = orangeBlob->avgX;
      newCand->centerY = orangeBlob->avgY;
      
      newCand->radius = ((float)(orangeBlob->avgX - orangeBlob->xi) + (float)(orangeBlob->xf - orangeBlob->avgX) + (float)(orangeBlob->avgY - orangeBlob->yi) + (float)(orangeBlob->yf - orangeBlob->avgY)) / 4.0;
      
      newCand->width = orangeBlob->dx;
      newCand->height = orangeBlob->dy;
      
      auto position = cmatrix_.getWorldPosition(newCand->centerX, newCand->centerY);
      newCand->groundDistance = cmatrix_.groundDistance(position);
      newCand->bearing = cmatrix_.bearing(position);
      newCand->elevation = cmatrix_.elevation(position);

      newCand->blob = orangeBlob;
      newCand->valid = true;

      // Allow factor of 3
      float expectedPixels = 3 * (39.5 - (4.43 * (RAD_T_DEG * newCand->elevation)) + (0.308 * pow((RAD_T_DEG * newCand->elevation), 2)));
      if (orangeBlob->total > expectedPixels) {
        //std::cout << "Eliminated for having unreasonable pixel to elevation relationship.\n";
        delete newCand;
        continue;
      }
      
      ballCands.push_back(newCand);
      
    }    
  }
}

bool BallDetector::checkBottomColor(Blob * orangeBlob) {
  if (camera_ == Camera::BOTTOM) {
    return checkSideColors(orangeBlob);
  }
  float rad = ((float)(orangeBlob->avgX - orangeBlob->xi) + (float)(orangeBlob->xf - orangeBlob->avgX) + (float)(orangeBlob->avgY - orangeBlob->yi) + (float)(orangeBlob->yf - orangeBlob->avgY)) / 4.0;
    
  // Make sure the ground below ball is green
  int floorY = (orangeBlob->yf + 4);
  // just in case, make sure the value is even
  floorY += (floorY % 2);

  if (floorY >= iparams_.height) {
    return checkSideColors(orangeBlob);
  }

  int floorX = orangeBlob->avgX;
  // Make sure that you are checking a valid point in the segmented image
  if (floorX % 4 != 0) floorX += (4 - (floorX % 4));
  // Don't go out of bounds of the picture
  if (floorX >= iparams_.width) {
    floorX -= 4;
  }
  
  unsigned char floorColor = getSegImg()[floorY * iparams_.width + floorX];

  // At the edges, the orange can appear pink, or the carpet can appear blue
  while ((floorColor == c_UNDEFINED || floorColor == c_PINK || floorColor == c_BLUE) && ((floorY - orangeBlob->yf) < rad)) {
    floorY+=2;

    if (floorY >= iparams_.height) {
      return checkSideColors(orangeBlob);
    }

    floorColor = getSegImg()[floorY * iparams_.width + floorX];
  }
  
  // if white, check the sides
  if (floorColor == c_WHITE) {
    return checkSideColors(orangeBlob);

    // Todo: what if the ball is horizontally on a white line?
    
  }
  else if (floorColor != c_FIELD_GREEN) {
    //std::cout << "no green below\n";
    return false;
  }
      
  return true;
}

bool BallDetector::checkSideColors(Blob * orangeBlob) {
  int floorY;
  if (camera_ == Camera::TOP) {
    floorY = (orangeBlob->yf < (iparams_.height - 2)) ? orangeBlob->yf : (iparams_.height - 2);
  }
  else
  {
    floorY = (int)(orangeBlob->avgY);
    floorY += (floorY % 2);
  }
  
  int lFloorX = orangeBlob->xi - 4;
  int rFloorX = orangeBlob->xf + 4;
  // Make sure that you are checking a valid point in the segmented image
  if (lFloorX % 4 != 0) lFloorX -= (4 - (lFloorX % 4));
  if (lFloorX < 0) lFloorX = 0;
  if (rFloorX % 4 != 0) rFloorX += (4 - (rFloorX % 4));
  if (rFloorX >= iparams_.width) rFloorX = (iparams_.width - 2);
  
  unsigned char rFloorColor = getSegImg()[floorY * iparams_.width + rFloorX];
  unsigned char lFloorColor = getSegImg()[floorY * iparams_.width + lFloorX];

  while ((lFloorColor == c_UNDEFINED || lFloorColor == c_PINK || lFloorColor == c_BLUE) && (lFloorX >= (orangeBlob->xi - 12))) {
    lFloorX-=4;

    if (lFloorX < 0) {
      lFloorX = 0;
    }

    lFloorColor = getSegImg()[floorY * iparams_.width + lFloorX];
  }
  
  while ((rFloorColor == c_UNDEFINED || rFloorColor == c_PINK || rFloorColor == c_BLUE) && (rFloorX <= (orangeBlob->xf + 12))) {
    rFloorX+=4;

    if (rFloorX > iparams_.width) {
      rFloorX = iparams_.width;
    }

    rFloorColor = getSegImg()[floorY * iparams_.width + rFloorX];
  }
  
  
  // Check either side of the ball
  if (rFloorColor != c_FIELD_GREEN && lFloorColor != c_FIELD_GREEN) {
    //std::cout << "Eliminated for not having green on either side\n";
    return false;
  }
  
  return true;
}

bool BallDetector::checkNextToLine(Blob * orangeBlob) {
  return checkSideColors(orangeBlob);
}

bool BallDetector::checkCenter(Blob * orangeBlob) {
  // Heuristic should favor small balls (center pixel should count twice!)
  // (Mostly exists to filter out decoys)
  if (orangeBlob->total < 100) return true;
  int numOrange = 0;
  int nonOrange = 0;
  float ratio = 0.75;
  unsigned char color;
  // Determine how many points to compare
  int numPoints = (std::min(orangeBlob->dx, orangeBlob->dy) / 4) - 1;
  if (numPoints % 2 != 1) numPoints--;
  if (numPoints < 3) numPoints = 3;

  // Determine where to check within the circle
  float minRad = std::min(std::min((float)orangeBlob->avgX - (float)orangeBlob->xi, (float)orangeBlob->xf - (float)orangeBlob->avgX), std::min((float)orangeBlob->avgY - (float)orangeBlob->yi, (float)orangeBlob->yf - (float)orangeBlob->avgY));

  float rFactor = 0.75;

  int yAxis = orangeBlob->avgY;
  if (orangeBlob->yf - orangeBlob->avgY > orangeBlob->avgY - orangeBlob->yi) {
    yAxis = yAxis + (yAxis %2);
  } else {
    yAxis = yAxis - (yAxis % 2);
  }
  
  int xStart = (int)((float)orangeBlob->avgX - rFactor * minRad);
  if ((xStart % 4) != 0) xStart += (4 - (xStart % 4));
  if (xStart < orangeBlob->xi) xStart = orangeBlob->xi;
  color = getSegImg()[yAxis * iparams_.width + xStart];

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;

  int xEnd = (int)((float)orangeBlob->avgX + rFactor * minRad);
  if ((xEnd % 4) != 0) xEnd -= (4 - (xEnd % 4));
  if (xEnd > orangeBlob->xf) xEnd = orangeBlob->xf;
  color = getSegImg()[yAxis * iparams_.width + xEnd];

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;

  float stepSize = (float)(xEnd - xStart) / (float)(numPoints - 1);

  // Now check the color value at each of these points
  for (int i = 0; i < (numPoints - 2); i++) {
    xStart += (int)stepSize;
    if ((xStart % 4) != 0) xStart += (4 - (xStart % 4));
    if (xStart > xEnd) xStart = xEnd;
    color = getSegImg()[yAxis * iparams_.width + xStart];

    ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;
  }

  int xAxis = orangeBlob->avgX;
  if (orangeBlob->xf - orangeBlob->avgX > orangeBlob->avgX - orangeBlob->yi) {
    xAxis = xAxis + (xAxis % 4);
  } else {
    xAxis = xAxis - (xAxis % 4);
  }
  
  int yStart = (int)((float)orangeBlob->avgY - rFactor * minRad);
  yStart += (yStart % 2);
  if (yStart < orangeBlob->yi) yStart = orangeBlob->yi;
  color = getSegImg()[yStart * iparams_.width + xAxis];

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;
  

  int yEnd = (int)((float)orangeBlob->avgY + rFactor * minRad);
  yEnd += (yEnd % 2);
  if (yEnd > orangeBlob->yf) yEnd = orangeBlob->yf;
  color = getSegImg()[yEnd * iparams_.width + yAxis];

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;

  stepSize = (float)(yEnd - yStart) / (float)(numPoints - 1);

  for (int i = 0; i < (numPoints - 2); i++) {
    yStart += (int)stepSize;
    yStart += (yStart % 2);
    if (yStart > yEnd) yStart = yEnd;
    color = getSegImg()[yStart * iparams_.width + xAxis];

    ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;
  }

  if (numOrange < (int)(ratio * (float)(2 * numPoints))) {
    return false;
  }
  return true;
}
