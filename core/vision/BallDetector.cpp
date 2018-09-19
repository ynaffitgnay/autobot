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

void BallDetector::findBall(std::vector<Blob>& blobs, std::vector<BallCandidate*>& ballCands) {
  if(camera_ == Camera::BOTTOM) return;
  
  static map<WorldObjectType,int> heights = {
    { WO_BALL, 33 },
  };
  
  auto fid = vblocks_.frame_info->frame_id;
  
  bool ballDetected = false;
  if(camera_ == Camera::TOP) {
    // TODO: fix this logic (right now we only detect one orange ball)
    Blob* orangeBlob = nullptr;
    BallCandidate* newCand = nullptr;
    int i = 0;
    for (auto blob : blobs) {
      if (blob.color == c_ORANGE) { //maybe update pixel ratio here too!!
        // std::cout << "frame: " << fid << " oBlob: " << i++ << " avgX: " << blob.avgX << " avgY: " << blob.avgY << " ToTaL: " << blob.total 
        //         << " xi: " << blob.xi << " yi: " << blob.yi << " xf: " << blob.xf << " yf: "
        //         << blob.yf << " pRatio: " << blob.correctPixelRatio << " pDensity: "
        //         << blob.pixelDensity << std::endl;
        orangeBlob = &blob;

        cv::Mat frame, grayFrame;

        int xPadding = std::max(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX);
        int yPadding = std::max(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY);
        float widthFactor = 1.25;
        float heightFactor = 1.25;
        
        // Make sure the row/col you want are within range
        int row = ((orangeBlob->yi - yPadding) < 0) ? 0 : (orangeBlob->yi - yPadding);
        int col = ((orangeBlob->xi - xPadding) < 0) ? 0 : (orangeBlob->xi - xPadding);
        int width = (int)((widthFactor * (float)(orangeBlob->xf - orangeBlob->xi + xPadding)) > iparams_.width) ? iparams_.width : (widthFactor * (float)(orangeBlob->xf - orangeBlob->xi + xPadding));
        int height = (int)(heightFactor * (float)(orangeBlob->yf - orangeBlob->yi + yPadding) > iparams_.height) ? iparams_.height : heightFactor * (float)(orangeBlob->yf - orangeBlob->yi + yPadding);

        grayFrame = color::rawToMatGraySubset(vblocks_.image->getImgTop(), iparams_, row, col, width, height, 1, 1);
        
        std::vector<cv::Vec3f> circles;
        
        if (!grayFrame.data) {
          return;
        }
        
        // Check to see if getting rid of this speeds anything up
        cv::GaussianBlur(grayFrame, grayFrame, cv::Size(13, 13), 0, 0);

        double maxRFactor = 2;
        double minRFactor = 0.65;
        double maxR = maxRFactor * (double)std::max(std::max(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX), std::max(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY));
        double minR = minRFactor * (double)std::min(std::min(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX), std::min(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY));
        double dp, p1, p2;
        // std::cout << " minR: " << (int)minR << " maxR: " << (int)maxR << " total: " << orangeBlob->total;// << "\n";

        //maxR = 0;
        //minR = 0;
        //minR = 1;
        //maxR = 15;

        if (orangeBlob->avgY < 63 && orangeBlob->total >= 10) {
          // std::cout << "tried to filter for decoy \n";
          continue;
        }

        if (orangeBlob->correctPixelRatio > 1.2) {
          // std::cout << "tried to filter for decoy \n";
          continue;
        }

        // INSTEAD OF THIS OR, CAN MAYBE LOOK AT GYRX TO SEE IF SITTING OR STANDING
        if (orangeBlob->avgY < 90 || (orangeBlob->avgY < 95 && orangeBlob->total < 20))
        {
          if (orangeBlob->total > 20) {
            continue;
          }
          dp = 2;
          p1 = 2;
          p2 = 2;
        //} else if (orangeBlob->avgY < 100) {
        //  dp = 5;
        //  p1 = 5;
        //  p2 = 10;
        } else if (orangeBlob->avgY < 110) {
          dp = 5;
          p1 = 10;
          p2 = 10;
        } else if (orangeBlob->avgY < 115) {
          dp = 4;
          p1 = 5;
          p2 = 10;
        } else if (orangeBlob->avgY < 135) {
          dp = 10;
          p1 = 10;
          p2 = 10;
        } else if (orangeBlob->avgY < 170) {
          dp = 6;
          p1 = 10;
          p2 = 10;
        } else if (orangeBlob->avgY > 200) {
          // minR = 0.75 * (double)std::min(std::min(orangeBlob->avgX - orangeBlob->xi, orangeBlob->xf - orangeBlob->avgX), std::min(orangeBlob->avgY - orangeBlob->yi, orangeBlob->yf - orangeBlob->avgY));
          dp = 5;
          p1 = 10;
          p2 = 10;
        } else {
          dp = 5;
          p1 = 10;
          p2 = 10;
        }

        // std:: cout << " dp: " << dp << " p1: " << p1 << " p2: " << p2 << "\n";
        //TODO: set smarter thresholds based off of size?
        cv::HoughCircles(grayFrame, circles, CV_HOUGH_GRADIENT, dp, grayFrame.rows/8, p1, p2, (int)minR, (int)maxR);
                
        for (size_t i = 0; i < circles.size(); ++i) {
          ballDetected = true;
          newCand = new BallCandidate;
          
          const cv::Vec3f& c = circles[i];
          //v[i][0] = c[0] + col; // if downsampled
          //v[i][1] = c[1] + row; // if downsampled
          //v[i][2] = c[2];

          newCand->centerX = c[0] + col;
          newCand->centerY = c[1] + row;
          newCand->radius = c[2];
          newCand->width = orangeBlob->dx;
          newCand->height = orangeBlob->dy;
          auto position = cmatrix_.getWorldPosition(newCand->centerX, newCand->centerY, heights[WO_BALL]);
          newCand->groundDistance = cmatrix_.groundDistance(position);
          newCand->elevation = cmatrix_.elevation(position);
          newCand->blob = orangeBlob;
          newCand->valid = true;
          //std::cout << "Circle " << i << " x: " << v[i][0] << " y: " << v[i][1] << " r: " << v[i][2] << "\n";
          ballCands.push_back(newCand);
          // std::cout << "newCand " << i << " x: " << newCand->centerX << " y: " << newCand->centerY << " r: " << newCand->radius << "\n\n\n\n\n\n";
        }

       
        //if(fid >= 6150) return;
        
        //auto& ball = vblocks_.world_object->objects_[WO_BALL];
        //
        //if (!ballDetected) return;
        //
        //ball.imageCenterX = v[0][0];
        //ball.imageCenterY = v[0][1];
        //ball.radius = v[0][2];
        //auto position = cmatrix_.getWorldPosition(ball.imageCenterX, ball.imageCenterY, heights[WO_BALL]);
        //ball.visionDistance = cmatrix_.groundDistance(position);
        //ball.visionBearing = cmatrix_.bearing(position);
        //ball.seen = true;
        //ball.fromTopCamera = camera_ == Camera::TOP;
        //tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_BALL), ball.imageCenterX, ball.imageCenterY, ball.visionDistance);
      }
    }
    
    //if (!orangeBlob) return;
    
    
    //}
  }
}
