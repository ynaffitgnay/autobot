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

void BallDetector::findBall(std::vector<Blob>& blobs) {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BALL, 33 },
  };
  static map<WorldObjectType,vector<int>> balls = {
    { WO_BALL, { 24, 15, 74, 83} }  //
  };
  auto fid = vblocks_.frame_info->frame_id;
  
  bool ballDetected = false;
  if(camera_ == Camera::TOP) {
    // TODO: fix this logic (right now we only detect one orange ball)
    Blob orangeBlob;
    
    for (auto blob : blobs) {
      if (blob.color == c_ORANGE) {
        orangeBlob = blob;
        break;
      }
    }
    
    if (orangeBlob.color != c_ORANGE) {
      return;
    }
    
    cv::Mat frame, grayFrame;

    int xPadding = std::max(orangeBlob.avgX - orangeBlob.xi, orangeBlob.xf - orangeBlob.avgX);
    int yPadding = std::max(orangeBlob.avgY - orangeBlob.yi, orangeBlob.yf - orangeBlob.avgY);
    float widthFactor = 1.25;
    float heightFactor = 1.25;
    
    // Make sure the row/col you want are within range
    int row = ((orangeBlob.yi - yPadding) < 0) ? 0 : (orangeBlob.yi - yPadding);
    int col = ((orangeBlob.xi - xPadding) < 0) ? 0 : (orangeBlob.xi - xPadding);
    int width = (int)((widthFactor * (float)(orangeBlob.xf - orangeBlob.xi + xPadding)) > iparams_.width) ? iparams_.width : (widthFactor * (float)(orangeBlob.xf - orangeBlob.xi + xPadding));
    int height = (int)(heightFactor * (float)(orangeBlob.yf - orangeBlob.yi + yPadding) > iparams_.height) ? iparams_.height : heightFactor * (float)(orangeBlob.yf - orangeBlob.yi + yPadding);
    
    grayFrame = color::rawToMatGraySubset(vblocks_.image->getImgTop(), iparams_, row, col, width, height, 1, 1);
    
    std::vector<cv::Vec3f> circles;
  
    if (!grayFrame.data) {
      return;
    }
    
    // Check to see if getting rid of this speeds anything up
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(9, 9), 0, 0);


    //TODO: set the min radius as 1/2 the min(orangeBlob.avgX - orangeBlob.xi, orangeBlob.xf - orangeBlob.avgX, orangeBlob.avgY - orangeBlob.yi, orangeBlob.yf - orangeBlob.avgY);g
    cv::HoughCircles(grayFrame, circles, CV_HOUGH_GRADIENT, 12, grayFrame.rows/8, 5, 10, 0, 0);
    
    // Now turn circles into a vector of floats
    // Populates a v with circles.size() elements, each a vector with 3 floats
    std::vector<std::vector<float>> v(circles.size(), std::vector<float>(3));
  
    for (size_t i = 0; i < circles.size(); ++i) {
      ballDetected = true;
      
      const cv::Vec3f& c = circles[i];
      v[i][0] = c[0] + col; // if downsampled
      v[i][1] = c[1] + row; // if downsampled
      v[i][2] = c[2];
  
      std::cout << "Circle " << i << " x: " << v[i][0] << " y: " << v[i][1] << " r: " << v[i][2] << "\n";
    }
  
    if(fid >= 6150) return;
    //for(auto ball : balls) {
    
    auto& ball = vblocks_.world_object->objects_[WO_BALL];
    //auto box = ball.second;
    //object.imageCenterX = (box[0] + box[2]) / 2;
    //object.imageCenterY = (box[1] + box[3]) / 2;
    
    if (!ballDetected) return;
    
    ball.imageCenterX = v[0][0];
    ball.imageCenterY = v[0][1];
    ball.radius = v[0][2];
    auto position = cmatrix_.getWorldPosition(ball.imageCenterX, ball.imageCenterY, heights[WO_BALL]);
    ball.visionDistance = cmatrix_.groundDistance(position);
    ball.visionBearing = cmatrix_.bearing(position);
    ball.seen = true;
    ball.fromTopCamera = camera_ == Camera::TOP;
    tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(WO_BALL), ball.imageCenterX, ball.imageCenterY, ball.visionDistance);
    
    //}
  }
}
