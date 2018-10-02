#include <vision/GoalDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>
#include <math/Common.h>

using namespace Eigen;
using namespace static_math;

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void GoalDetector::findGoals(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  if(blobs.size() <= 0) return;
  
  static map<WorldObjectType,int> heights = {
    { WO_UNKNOWN_GOAL, 253 }
  };
  
  float b = 330;
  float correctAspectRatio = 1.8;
  Blob* goalBlob = nullptr;
  bool skewedPositive;
  
  if((blobs.at(0).color == c_BLUE) && (blobs.at(0).lpCount>=10) && (blobs.at(0).total>=500)) {
    goalBlob = &(blobs.at(0));
        
    auto& goal = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    goal.imageCenterY = goalBlob->avgY;
    float aspectRatio = ((float) goalBlob->dx)/((float) goalBlob->dy);
    // std::cout << "Goal Found!" << std::endl;
    // printf("Width = %d, Height = %d, Width/Height = %f\n", goalBlob->dx,  goalBlob->dy, aspectRatio);

    // If not skewed
    if (aspectRatio>1.6 && aspectRatio<2.25)
    {
      std::cout << "Not skewed!\n";
      goal.orientation = 0.0;
      goal.imageCenterX = goalBlob->avgX;
      auto position = cmatrix_.getWorldPosition(goal.imageCenterX, goal.imageCenterY, heights[WO_UNKNOWN_GOAL]);
      goal.visionDistance = cmatrix_.groundDistance(position);
      goal.visionBearing = cmatrix_.bearing(position);
      goal.visionElevation = cmatrix_.elevation(position);
      goal.seen = true;
      goal.fromTopCamera = camera_ == Camera::TOP;
      tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_UNKNOWN_GOAL), goal.imageCenterX, goal.imageCenterY, goal.visionDistance);
    }
    // Else, if aspectRatio smaller (skewed)
    else if (aspectRatio <=1.6) {
      skewedPositive = goalSkewedPos(goalBlob); 
      goal.orientation = acosf(aspectRatio/correctAspectRatio);

      if (!skewedPositive) goal.orientation = goal.orientation * -1.0;
      
      float s_theta = sqrtf(1.0 - aspectRatio * aspectRatio / correctAspectRatio / correctAspectRatio);

      auto pos = cmatrix_.getWorldPosition(goalBlob->avgX,goalBlob->avgY, heights[WO_UNKNOWN_GOAL]);
      float b_theta = cmatrix_.getCameraWidthByDistance(cmatrix_.groundDistance(pos), b * s_theta);
      
      // std::cout << "b sin(theta) into image frame: " << b_theta << std::endl;
      
      if (skewedPositive)
      {
        // std::cout << "Skewed positive (reduce centerX)\n";
        goal.imageCenterX = goalBlob->avgX - b_theta;
        if (goal.imageCenterX < 0) goal.imageCenterX = 0;
      } else {
        // std::cout << "Skewed negative (increase centerX)\n";
        goal.imageCenterX = goalBlob->avgX + b_theta;
        if (goal.imageCenterX > iparams_.width - 2) goal.imageCenterX = iparams_.width - 2;
      }
      auto position = cmatrix_.getWorldPosition(goal.imageCenterX, goal.imageCenterY, heights[WO_UNKNOWN_GOAL]);
      goal.visionDistance = cmatrix_.groundDistance(position);
      //std::cout << "Skewed goal distance: " << goal.visionDistance << std::endl;
      goal.seen = true;
      goal.fromTopCamera = camera_ == Camera::TOP;
      tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_UNKNOWN_GOAL), goal.imageCenterX, goal.imageCenterY, goal.visionDistance);
    }
    else {
      goal.seen = false;
    }
  }
  else {
    auto& goal = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    goal.seen = false;
    // std::cout << "No Goal here!" << std::endl;
  }
}

unsigned char* GoalDetector::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

bool GoalDetector::goalSkewedPos(Blob* goalBlob) {
  int thresholdForCutoff = 5;
  float forceStopCount = ((float)goalBlob->dy / 2.5);
  int leftX = goalBlob->xi;
  int rightX = goalBlob->xf;
  int y, greenCount;
  unsigned char floorColor;
  int leftWhiteCount = 0;
  int rightWhiteCount = 0;

  int finalY = (goalBlob->yf + (int)forceStopCount < iparams_.height - 2) ? goalBlob->yf + (int)forceStopCount : iparams_.height - 2;
  finalY -= (finalY % 2);  // Make sure you are checking a valid point in the segmented image

  if (leftX == 0) {
    leftWhiteCount = thresholdForCutoff;
  } else {
    greenCount = 0;
    
    for (y = goalBlob->yf; y <= finalY; y+=2) {
      if (greenCount >= 3) {
        break;
      }

      floorColor = getSegImg()[y * iparams_.width + leftX];
      
      if (floorColor == c_WHITE || floorColor == c_ROBOT_WHITE) {
        leftWhiteCount++;
      } else if (floorColor == c_FIELD_GREEN) {
        greenCount++;
      }
    }
  }

  if (rightX >= iparams_.width - 2) {
    rightWhiteCount = thresholdForCutoff;
  } else {
    greenCount = 0;
    
    for (y = goalBlob->yf; y <= finalY; y+=2) {
      if (greenCount >= 3) {
        break;
      }

      floorColor = getSegImg()[y * iparams_.width + rightX];
      
      if (floorColor == c_WHITE || floorColor == c_ROBOT_WHITE) {
        rightWhiteCount++;
      } else if (floorColor == c_FIELD_GREEN) {
        greenCount++;
      }
    }
  }

  if (rightWhiteCount > leftWhiteCount) {
    return false;
  }

  return true;
}
