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
  Blob* otherGoalBlob = nullptr;
  Blob mergedBlob;
  int distbetween = 0;
  //bool checkOtherBlob = true;
  bool skewedPositive;
  
  if((blobs.at(0).color == c_BLUE) && (blobs.at(0).lpCount>=10) && (blobs.at(0).total>=500)) {
    goalBlob = &(blobs.at(0));

    // Look for the rest of the goal
    std::vector<Blob>::iterator blobIt = blobs.begin();
    blobIt++;
    for(blobIt; blobIt != blobs.end(); blobIt++)
    {
      if (blobIt->color == c_BLUE)
      {
        otherGoalBlob = &(*blobIt);
        break;
      }
    }

    if (otherGoalBlob != nullptr)
    {
      //std::cout << "fid: " << vblocks_.frame_info->frame_id << std::endl;
      //std::cout << "goal avgX: " << goalBlob->avgX << " avgY: " << goalBlob->avgY << " ToTaL: " << goalBlob->total 
      //          << " xi: " << goalBlob->xi << " yi: " << goalBlob->yi << " xf: " << goalBlob->xf << " yf: "
      //          << goalBlob->yf << " dy: " << goalBlob->dy << std::endl;
      //std::cout << "other avgX: " << otherGoalBlob->avgX << " avgY: " << otherGoalBlob->avgY << " ToTaL: " << otherGoalBlob->total 
      //          << " xi: " << otherGoalBlob->xi << " yi: " << otherGoalBlob->yi << " xf: " << otherGoalBlob->xf << " yf: "
      //          << otherGoalBlob->yf << " dy: " << otherGoalBlob->dy << std::endl;

      if (goalBlob->xf < otherGoalBlob->xi)
      {
        distbetween = otherGoalBlob->xi - goalBlob->xf;
        // merge goal blobs?
        mergedBlob.dx = otherGoalBlob->xf - goalBlob->xi;
      }
      else if (otherGoalBlob->xf < goalBlob->xi)
      {
        distbetween = goalBlob->xi - otherGoalBlob->xf;
        // merge goal blobs?
        mergedBlob.dx = goalBlob->xf - otherGoalBlob->xi;
      }
      else
      {
        distbetween = 0;
        //std::cout << "Overlapping goal blobs?\n";
        // do more here
        // TODO: FIX THIS

        mergedBlob.avgX = goalBlob->avgX;
        mergedBlob.avgY = goalBlob->avgY;
        mergedBlob.dx = goalBlob->dx;
        mergedBlob.dy = goalBlob->dy;
        
        // if overlapping, fix the avgY
        if (otherGoalBlob->avgY >= goalBlob->yi && otherGoalBlob->avgY <= goalBlob->yf)
        {
          mergedBlob.avgY = 0.5 * (goalBlob->yi + goalBlob->yf);
          // TODO: FIX
          //goalBlob->avgY = mergedBlob.avgY;
          //std::cout << "updated avgY to " << mergedBlob.avgY << std::endl;
        }
        else
        {
          //std::cout << "did not update avgY\n";
        }
        goalBlob = &mergedBlob;
        //mergedBlob.avgX = 0.5 * (goalBlob->xi + goalBlob->xf);

        //auto newposition = cmatrix_.getWorldPosition(mergedBlob.avgX, mergedBlob.avgY, heights[WO_UNKNOWN_GOAL]);
        //std::cout << "Merged distance would be: " << cmatrix_.groundDistance(newposition) << std::endl;
      }
      
      //std::cout << "Orig dy: " << goalBlob->dy << ", our dy: " << otherGoalBlob->dy << ", dist between: " << distbetween << std::endl;
      if (distbetween <= 60 && std::abs(otherGoalBlob->dy - goalBlob->dy) <= 14)
      {
        //std::cout << "Merging blobs!" << std::endl;
        mergedBlob.avgX = ((float)otherGoalBlob->total / ((float)otherGoalBlob->total + (float)goalBlob->total)) * (float)otherGoalBlob->avgX +
          ((float)goalBlob->total / ((float)otherGoalBlob->total + (float)goalBlob->total)) * (float)goalBlob->avgX;
      
        mergedBlob.avgY = (otherGoalBlob->avgY > goalBlob->avgY) ? otherGoalBlob->avgY : goalBlob->avgY;
        mergedBlob.dy = (otherGoalBlob->dy > goalBlob->dy) ? otherGoalBlob->dy : goalBlob->dy;
        goalBlob = &mergedBlob;
      
        //std::cout << "merged avgX: " << mergedBlob.avgX << " avgY: " << mergedBlob.avgY << " ToTaL: " << mergedBlob.total 
        //          << " xi: " << mergedBlob.xi << " yi: " << mergedBlob.yi << " xf: " << mergedBlob.xf << " yf: "
        //          << mergedBlob.yf << " dy: " << mergedBlob.dy << std::endl;
      
      }
    }

    

    

    auto& goal = vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
    
    float aspectRatio = ((float) goalBlob->dx)/((float) goalBlob->dy);
    // std::cout << "Goal Found!" << std::endl;
    // printf("Width = %d, Height = %d, Width/Height = %f\n", goalBlob->dx,  goalBlob->dy, aspectRatio);

    goal.imageCenterY = goalBlob->avgY;
    // If not skewed
    //if (aspectRatio>1.6 && aspectRatio<2.25)
    if (aspectRatio < 2.25)
    {
      // std::cout << "Not skewed!\n";
      goal.orientation = 0.0;
      goal.imageCenterX = goalBlob->avgX;
      auto position = cmatrix_.getWorldPosition(goal.imageCenterX, goal.imageCenterY, heights[WO_UNKNOWN_GOAL]);
      goal.visionDistance = cmatrix_.groundDistance(position);
      goal.visionBearing = cmatrix_.bearing(position);
      goal.visionElevation = cmatrix_.elevation(position);
      goal.seen = true;
      goal.fromTopCamera = camera_ == Camera::TOP;
      tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_UNKNOWN_GOAL), goal.imageCenterX, goal.imageCenterY, goal.visionDistance);
      // std::cout << "Goal distance: " << goal.visionDistance << std::endl;
    }
    // Else, if aspectRatio smaller (skewed)
    //else if (aspectRatio <=1.6) {
    //  skewedPositive = goalSkewedPos(goalBlob); 
    //  goal.orientation = acosf(aspectRatio/correctAspectRatio);
    //
    //  if (!skewedPositive) goal.orientation = goal.orientation * -1.0;
    //  
    //  float s_theta = sqrtf(1.0 - aspectRatio * aspectRatio / correctAspectRatio / correctAspectRatio);
    //
    //  auto pos = cmatrix_.getWorldPosition(goalBlob->avgX,goalBlob->avgY, heights[WO_UNKNOWN_GOAL]);
    //  float b_theta = cmatrix_.getCameraWidthByDistance(cmatrix_.groundDistance(pos), b * s_theta);
    //  
    //  // std::cout << "b sin(theta) into image frame: " << b_theta << std::endl;
    //  
    //  if (skewedPositive)
    //  {
    //    std::cout << "Skewed positive (reduce centerX)\n";
    //    goal.imageCenterX = goalBlob->avgX - b_theta;
    //    if (goal.imageCenterX < 0) goal.imageCenterX = 0;
    //  } else {
    //    std::cout << "Skewed negative (increase centerX)\n";
    //    goal.imageCenterX = goalBlob->avgX + b_theta;
    //    if (goal.imageCenterX > iparams_.width - 2) goal.imageCenterX = iparams_.width - 2;
    //  }
    //  auto position = cmatrix_.getWorldPosition(goal.imageCenterX, goal.imageCenterY, heights[WO_UNKNOWN_GOAL]);
    //  goal.visionDistance = cmatrix_.groundDistance(position);
    //  std::cout << "Skewed goal distance: " << goal.visionDistance << std::endl;
    //  goal.seen = true;
    //  goal.visionBearing = cmatrix_.bearing(position);
    //  goal.visionDistance = cmatrix_.groundDistance(position);
    //  goal.visionElevation = cmatrix_.elevation(position);
    //  goal.fromTopCamera = camera_ == Camera::TOP;
    //  tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_UNKNOWN_GOAL), goal.imageCenterX, goal.imageCenterY, goal.visionDistance);
    //}
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
