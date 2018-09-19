#include <vision/BallDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>
#include <vision/ImageProcessor.h>
#include <vision/structures/Blob.h>
#include <common/ColorConversion.h>
#include <iostream>
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
  int ballHeightMM = 40;
  
  auto fid = vblocks_.frame_info->frame_id;
  
  bool ballDetected = false;
  // TODO: fix this logic (right now we only detect one orange ball)
  Blob* orangeBlob = nullptr;
  BallCandidate* newCand = nullptr;
  int i = 0;
  for (auto blob : blobs) {
    if (blob.color == c_ORANGE) { //maybe update pixel ratio here too!!
      if (camera_ == Camera::TOP) {
        //std::cout << "top ";
      } else {
        //std::cout << "bottom ";
      }
      //std::cout << "frame: " << fid << " oBlob: " << i++ << " avgX: " << blob.avgX << " avgY: " << blob.avgY << " ToTaL: " << blob.total 
      //          << " xi: " << blob.xi << " yi: " << blob.yi << " xf: " << blob.xf << " yf: "
      //          << blob.yf << " pRatio: " << blob.correctPixelRatio << " pDensity: "
      //          << blob.pixelDensity << std::endl;

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
      
      float ratioHighFactor = 1.45;  // maybe add heuristics for higher pixel ratio when in lower quadrant
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


      // Make sure that within a certain Y value, radius is not greater than ???
      
      newCand = new BallCandidate;
      
      newCand->centerX = orangeBlob->avgX;
      newCand->centerY = orangeBlob->avgY;
      
      newCand->radius = ((float)(orangeBlob->avgX - orangeBlob->xi) + (float)(orangeBlob->xf - orangeBlob->avgX) + (float)(orangeBlob->avgY - orangeBlob->yi) + (float)(orangeBlob->yf - orangeBlob->avgY)) / 4.0;
      
      newCand->width = orangeBlob->dx;
      newCand->height = orangeBlob->dy;
      
      auto position = cmatrix_.getWorldPosition(newCand->centerX, newCand->centerY, ballHeightMM);
      newCand->groundDistance = cmatrix_.groundDistance(position);


      // CHECK ELEVATION
      newCand->bearing = cmatrix_.bearing(position);
      newCand->elevation = cmatrix_.elevation(position);
      newCand->blob = orangeBlob;
      newCand->valid = true;

      ////std::cout << "X: " << newCand->centerX << " Y: " << newCand->centerY << " R: " << newCand->radius <<
      //  " groundDist: " << newCand->groundDistance << " elevation: " << newCand-> elevation << "\n";

      // if (elevation check)
      // delete newCand;
      //
      ballCands.push_back(newCand);

      
      //cv::Mat frame, grayFrame;
      //
      //int xPadding = std::max(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX);
      //int yPadding = std::max(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY);
      //float widthFactor = 1.25;
      //float heightFactor = 1.25;
      //
      //// Make sure the row/col you want are within range
      //int row = ((orangeBlob->yi - yPadding) < 0) ? 0 : (orangeBlob->yi - yPadding);
      //int col = ((orangeBlob->xi - xPadding) < 0) ? 0 : (orangeBlob->xi - xPadding);
      //int width = (int)((widthFactor * (float)(orangeBlob->xf - orangeBlob->xi + xPadding)) > iparams_.width) ? iparams_.width : (widthFactor * (float)(orangeBlob->xf - orangeBlob->xi + xPadding));
      //int height = (int)(heightFactor * (float)(orangeBlob->yf - orangeBlob->yi + yPadding) > iparams_.height) ? iparams_.height : heightFactor * (float)(orangeBlob->yf - orangeBlob->yi + yPadding);
      //
      //grayFrame = color::rawToMatGraySubset(getImg(), iparams_, row, col, width, height, 1, 1);
      //
      //std::vector<cv::Vec3f> circles;
      //
      //if (!grayFrame.data) {
      //  return;
      //}
      //
      //// Check to see if getting rid of this speeds anything up
      //cv::GaussianBlur(grayFrame, grayFrame, cv::Size(13, 13), 0, 0);
      //
      //double maxRFactor = 2;
      //double minRFactor = 0.65;
      //double maxR = maxRFactor * (double)std::max(std::max(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX), std::max(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY));
      //double minR = minRFactor * (double)std::min(std::min(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX), std::min(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY));
      //double dp, p1, p2;
      //// //std::cout << " minR: " << (int)minR << " maxR: " << (int)maxR << " total: " << orangeBlob->total;// << "\n";
      //
      //if (orangeBlob->avgY < 63 && orangeBlob->total >= 10) {
      //  // //std::cout << "tried to filter for decoy \n";
      //  continue;
      //}
      //
      //if (orangeBlob->correctPixelRatio > 1.2) {
      //  // //std::cout << "tried to filter for decoy \n";
      //  continue;
      //}
      //
      //// INSTEAD OF THIS OR, CAN MAYBE LOOK AT GYRX TO SEE IF SITTING OR STANDING
      //if (orangeBlob->avgY < 90 || (orangeBlob->avgY < 95 && orangeBlob->total < 20))
      //{
      //  if (orangeBlob->total > 20) {
      //    continue;
      //  }
      //  dp = 2;
      //  p1 = 2;
      //  p2 = 2;
      ////} else if (orangeBlob->avgY < 100) {
      ////  dp = 5;
      ////  p1 = 5;
      ////  p2 = 10;
      //} else if (orangeBlob->avgY < 110) {
      //  dp = 5;
      //  p1 = 10;
      //  p2 = 10;
      //} else if (orangeBlob->avgY < 115) {
      //  dp = 4;
      //  p1 = 5;
      //  p2 = 10;
      //} else if (orangeBlob->avgY < 135) {
      //  dp = 10;
      //  p1 = 10;
      //  p2 = 10;
      //} else if (orangeBlob->avgY < 170) {
      //  dp = 6;
      //  p1 = 10;
      //  p2 = 10;
      //} else if (orangeBlob->avgY > 200) {
      //  // minR = 0.75 * (double)std::min(std::min(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX), std::min(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY));
      //  dp = 5;
      //  p1 = 10;
      //  p2 = 10;
      //} else {
      //  dp = 5;
      //  p1 = 10;
      //  p2 = 10;
      //}
      //
      //// std:: cout << " dp: " << dp << " p1: " << p1 << " p2: " << p2 << "\n";
      ////TODO: set smarter thresholds based off of size?
      //cv::HoughCircles(grayFrame, circles, CV_HOUGH_GRADIENT, dp, grayFrame.rows/8, p1, p2, (int)minR, (int)maxR);
      //        
      //for (size_t i = 0; i < circles.size(); ++i) {
      //  ballDetected = true;
      //  newCand = new BallCandidate;
      //  
      //  const cv::Vec3f& c = circles[i];
      //  //v[i][0] = c[0] + col; // if downsampled
      //  //v[i][1] = c[1] + row; // if downsampled
      //  //v[i][2] = c[2];
      //
      //  newCand->centerX = c[0] + col;
      //  newCand->centerY = c[1] + row;
      //  newCand->radius = c[2];
      //  newCand->width = orangeBlob->dx;
      //  newCand->height = orangeBlob->dy;
      //  auto position = cmatrix_.getWorldPosition(newCand->centerX, newCand->centerY, ballHeightMM);
      //  newCand->groundDistance = cmatrix_.groundDistance(position);
      //  newCand->elevation = cmatrix_.elevation(position);
      //  newCand->blob = orangeBlob;
      //  newCand->valid = true;
      //  ////std::cout << "Circle " << i << " x: " << v[i][0] << " y: " << v[i][1] << " r: " << v[i][2] << "\n";
      //  ballCands.push_back(newCand);
      //  // //std::cout << "newCand " << i << " x: " << newCand->centerX << " y: " << newCand->centerY << " r: " << newCand->radius << "\n\n\n\n\n\n";
      //}
    }    
  }
}

bool BallDetector::checkBottomColor(Blob * orangeBlob) {
  if (camera_ == Camera::BOTTOM) {
    return checkSideColors(orangeBlob);
  }
  float rad = ((float)(orangeBlob->avgX - orangeBlob->xi) + (float)(orangeBlob->xf - orangeBlob->avgX) + (float)(orangeBlob->avgY - orangeBlob->yi) + (float)(orangeBlob->yf - orangeBlob->avgY)) / 4.0;
    
  // Make sure the ground below ball is green
  //int floorY = (int)(0.10 * (float)(orangeBlob->dy) + (float)orangeBlob->yf);
  //if ((floorY % 2) != 0) floorY += 1;
  //int incFloor = 2;
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

    ////std::cout << "updating floorColor\n";
    floorColor = getSegImg()[floorY * iparams_.width + floorX];
  }
  
//  //std::cout <<  " floorX: " << floorX << " floorY: " << floorY << " color: ";
//
//
//  if (floorColor == c_UNDEFINED) {
//    //std::cout << " UNDEFINED";
//  } else if (floorColor == c_FIELD_GREEN) {
//    //std::cout << " GREEN";
//  } else if (floorColor == c_WHITE) {           
//    //std::cout << " WHITE";
//  } else if (floorColor == c_ORANGE) {          
//    //std::cout << " ORANGE";
//  } else if (floorColor == c_PINK) {            
//    //std::cout << " PINK";
//  } else if (floorColor == c_BLUE) {            
//    //std::cout << " BLUE";
//  } else if (floorColor == c_YELLOW) {          
//    //std::cout << " YELLOW";
//  } else if (floorColor == c_ROBOT_WHITE) {     
//    //std::cout << " ROBOT";
//  } else {
//    //std::cout << "what??";
//  }
//  //std::cout << "\n";

  

  // if white, check the sides
  if (floorColor == c_WHITE) {
    // check the ground on either side of the ball
    // floorY = (orangeBlob->yf + incFloor);
    // just in case, make sure the value is even
    //floorY += (floorY % 2);

    //int lFloorX = orangeBlob->xi;
    //int rFloorX = orangeBlob->xf;
    //// Make sure that you are checking a valid point in the segmented image
    //if (floorX % 4 != 0) floorX += (4 - (floorX % 4));
    //
    //unsigned char rFloorColor = getSegImg()[floorY * iparams_.width + rFloorX];
    //unsigned char lFloorColor = getSegImg()[floorY * iparams_.width + lFloorX];
    //
    //// Check either side of the ball
    //if (rFloorColor != c_FIELD_GREEN && lFloorColor != c_FIELD_GREEN) {
    //  //std::cout << "Eliminated over white line with no green\n";
    //  continue;
    //}
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
  //int floorY = std::min(orangeBlob->yf, (iparams_.height - 2));
  int floorY;
  // Check from the bottom of the ball if using the top camera
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

    ////std::cout << "updating rFloorColor\n";
    lFloorColor = getSegImg()[floorY * iparams_.width + lFloorX];
  }
  
  while ((rFloorColor == c_UNDEFINED || rFloorColor == c_PINK || rFloorColor == c_BLUE) && (rFloorX <= (orangeBlob->xf + 12))) {
    rFloorX+=4;

    if (rFloorX > iparams_.width) {
      rFloorX = iparams_.width;
    }

    ////std::cout << "updating rFloorColor\n";
    rFloorColor = getSegImg()[floorY * iparams_.width + rFloorX];
  }
  

// //std::cout << "rFloorColor: ";
// if (rFloorColor == c_UNDEFINED) {
//   //std::cout << " UNDEFINED";
// } else if (rFloorColor == c_FIELD_GREEN) {
//   //std::cout << " GREEN";
// } else if (rFloorColor == c_WHITE) {           
//   //std::cout << " WHITE";
// } else if (rFloorColor == c_ORANGE) {          
//   //std::cout << " ORANGE";
// } else if (rFloorColor == c_PINK) {            
//   //std::cout << " PINK";
// } else if (rFloorColor == c_BLUE) {            
//   //std::cout << " BLUE";
// } else if (rFloorColor == c_YELLOW) {          
//   //std::cout << " YELLOW";
// } else if (rFloorColor == c_ROBOT_WHITE) {     
//   //std::cout << " ROBOT";
// } else {
//   //std::cout << "what??";
// }
// //std::cout << "  lFloorColor: ";
// if (lFloorColor == c_UNDEFINED) {
//   //std::cout << " UNDEFINED";
// } else if (lFloorColor == c_FIELD_GREEN) {
//   //std::cout << " GREEN";
// } else if (lFloorColor == c_WHITE) {           
//   //std::cout << " WHITE";
// } else if (lFloorColor == c_ORANGE) {          
//   //std::cout << " ORANGE";
// } else if (lFloorColor == c_PINK) {            
//   //std::cout << " PINK";
// } else if (lFloorColor == c_BLUE) {            
//   //std::cout << " BLUE";
// } else if (lFloorColor == c_YELLOW) {          
//   //std::cout << " YELLOW";
// } else if (lFloorColor == c_ROBOT_WHITE) {     
//   //std::cout << " ROBOT";
// } else {
//   //std::cout << "what??";
// }
// //std::cout << "\n";
//

  
  // Check either side of the ball
  if (rFloorColor != c_FIELD_GREEN && lFloorColor != c_FIELD_GREEN) {
    ////std::cout << "Eliminated over white line with no green\n";
    //std::cout << "Eliminated for not having green on either side\n";
    return false;
  }
  
  return true;
}

bool BallDetector::checkNextToLine(Blob * orangeBlob) {
  //todo: add more logic, if necessary
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

  //std::cout << "numPoints: " << numPoints;

  // Determine where to check within the circle
  float minRad = std::min(std::min((float)orangeBlob->avgX - (float)orangeBlob->xi, (float)orangeBlob->xf - (float)orangeBlob->avgX), std::min((float)orangeBlob->avgY - (float)orangeBlob->yi, (float)orangeBlob->yf - (float)orangeBlob->avgY));
  //std::cout << " minRad: " << minRad << "\n";

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
//  //std::cout << "color: ";
//  if (color == c_UNDEFINED) {
//    //std::cout << " UNDEFINED";
//  } else if (color == c_FIELD_GREEN) {
//    //std::cout << " GREEN";
//  } else if (color == c_WHITE) {           
//    //std::cout << " WHITE";
//  } else if (color == c_ORANGE) {          
//    //std::cout << " ORANGE";
//  } else if (color == c_PINK) {            
//    //std::cout << " PINK";
//  } else if (color == c_BLUE) {            
//    //std::cout << " BLUE";
//  } else if (color == c_YELLOW) {          
//    //std::cout << " YELLOW";
//  } else if (color == c_ROBOT_WHITE) {     
//    //std::cout << " ROBOT";
//  } else {
//    //std::cout << "what??";
//  }
//  //std::cout << "\n";

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;

  int xEnd = (int)((float)orangeBlob->avgX + rFactor * minRad);
  if ((xEnd % 4) != 0) xEnd -= (4 - (xEnd % 4));
  if (xEnd > orangeBlob->xf) xEnd = orangeBlob->xf;
  color = getSegImg()[yAxis * iparams_.width + xEnd];
//    //std::cout << "color: ";
//  if (color == c_UNDEFINED) {
//    //std::cout << " UNDEFINED";
//  } else if (color == c_FIELD_GREEN) {
//    //std::cout << " GREEN";
//  } else if (color == c_WHITE) {           
//    //std::cout << " WHITE";
//  } else if (color == c_ORANGE) {          
//    //std::cout << " ORANGE";
//  } else if (color == c_PINK) {            
//    //std::cout << " PINK";
//  } else if (color == c_BLUE) {            
//    //std::cout << " BLUE";
//  } else if (color == c_YELLOW) {          
//    //std::cout << " YELLOW";
//  } else if (color == c_ROBOT_WHITE) {     
//    //std::cout << " ROBOT";
//  } else {
//    //std::cout << "what??";
//  }
//  //std::cout << "\n";

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;

  float stepSize = (float)(xEnd - xStart) / (float)(numPoints - 1);

  // Now check the color value at each of these points
  for (int i = 0; i < (numPoints - 2); i++) {
    xStart += (int)stepSize;
    if ((xStart % 4) != 0) xStart += (4 - (xStart % 4));
    if (xStart > xEnd) xStart = xEnd;
    color = getSegImg()[yAxis * iparams_.width + xStart];
//    //std::cout << "color: ";
//    if (color == c_UNDEFINED) {
//      //std::cout << " UNDEFINED";
//    } else if (color == c_FIELD_GREEN) {
//      //std::cout << " GREEN";
//    } else if (color == c_WHITE) {           
//      //std::cout << " WHITE";
//    } else if (color == c_ORANGE) {          
//      //std::cout << " ORANGE";
//    } else if (color == c_PINK) {            
//      //std::cout << " PINK";
//    } else if (color == c_BLUE) {            
//      //std::cout << " BLUE";
//    } else if (color == c_YELLOW) {          
//      //std::cout << " YELLOW";
//    } else if (color == c_ROBOT_WHITE) {     
//      //std::cout << " ROBOT";
//    } else {
//      //std::cout << "what??";
//    }
//    //std::cout << "\n";

    ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;
  }

  int xAxis = orangeBlob->avgX;
  if (orangeBlob->xf - orangeBlob->avgX > orangeBlob->avgX - orangeBlob->yi) {
    xAxis = xAxis + (xAxis % 4);
  } else {
    xAxis = xAxis - (xAxis % 4);
  }
  
  int yStart = (int)((float)orangeBlob->avgY - rFactor * minRad);
  //if ((yStart % 4) != 0) yStart += (4 - (yStart % 4));
  yStart += (yStart % 2);
  if (yStart < orangeBlob->yi) yStart = orangeBlob->yi;
  color = getSegImg()[yStart * iparams_.width + xAxis];
//  //std::cout << "color: ";
//  if (color == c_UNDEFINED) {
//    //std::cout << " UNDEFINED";
//  } else if (color == c_FIELD_GREEN) {
//    //std::cout << " GREEN";
//  } else if (color == c_WHITE) {           
//    //std::cout << " WHITE";
//  } else if (color == c_ORANGE) {          
//    //std::cout << " ORANGE";
//  } else if (color == c_PINK) {            
//    //std::cout << " PINK";
//  } else if (color == c_BLUE) {            
//    //std::cout << " BLUE";
//  } else if (color == c_YELLOW) {          
//    //std::cout << " YELLOW";
//  } else if (color == c_ROBOT_WHITE) {     
//    //std::cout << " ROBOT";
//  } else {
//    //std::cout << "what??";
//  }
//  //std::cout << "\n";

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;
  

  int yEnd = (int)((float)orangeBlob->avgY + rFactor * minRad);
  yEnd += (yEnd % 2);
  if (yEnd > orangeBlob->yf) yEnd = orangeBlob->yf;
  color = getSegImg()[yEnd * iparams_.width + yAxis];
//  //std::cout << "color: ";
//  if (color == c_UNDEFINED) {
//    //std::cout << " UNDEFINED";
//  } else if (color == c_FIELD_GREEN) {
//    //std::cout << " GREEN";
//  } else if (color == c_WHITE) {           
//    //std::cout << " WHITE";
//  } else if (color == c_ORANGE) {          
//    //std::cout << " ORANGE";
//  } else if (color == c_PINK) {            
//    //std::cout << " PINK";
//  } else if (color == c_BLUE) {            
//    //std::cout << " BLUE";
//  } else if (color == c_YELLOW) {          
//    //std::cout << " YELLOW";
//  } else if (color == c_ROBOT_WHITE) {     
//    //std::cout << " ROBOT";
//  } else {
//    //std::cout << "what??";
//  }
//  //std::cout << "\n";

  ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;

  stepSize = (float)(yEnd - yStart) / (float)(numPoints - 1);

  for (int i = 0; i < (numPoints - 2); i++) {
    yStart += (int)stepSize;
    yStart += (yStart % 2);
    if (yStart > yEnd) yStart = yEnd;
    color = getSegImg()[yStart * iparams_.width + xAxis];
//    //std::cout << "color: ";
//    if (color == c_UNDEFINED) {
//      //std::cout << " UNDEFINED";
//    } else if (color == c_FIELD_GREEN) {
//      //std::cout << " GREEN";
//    } else if (color == c_WHITE) {           
//      //std::cout << " WHITE";
//    } else if (color == c_ORANGE) {          
//      //std::cout << " ORANGE";
//    } else if (color == c_PINK) {            
//      //std::cout << " PINK";
//    } else if (color == c_BLUE) {            
//      //std::cout << " BLUE";
//    } else if (color == c_YELLOW) {          
//      //std::cout << " YELLOW";
//    } else if (color == c_ROBOT_WHITE) {     
//      //std::cout << " ROBOT";
//    } else {
//      //std::cout << "what??";
//    }
//    //std::cout << "\n";

    ((color == c_ORANGE) || (color == c_UNDEFINED)) ? numOrange++ : nonOrange++;
  }


  //std::cout << " numOrange: " << numOrange << " nonOrange: " << nonOrange << "\n";

  if (numOrange < (int)(ratio * (float)(2 * numPoints))) {
    return false;
  }
  return true;
}
