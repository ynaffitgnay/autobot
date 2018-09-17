#include <vision/BallDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>
#include <vision/structures/Blob.h>
#include <common/ColorConversion.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace Eigen;

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void BallDetector::detectBall(std::vector<Blob>& blobs) {
  int imageX, imageY;
  if(!findBall2(blobs, imageX,imageY)) return; // findBall fills in imageX and imageY
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  // Now we know where the ball's position
  ball->seen = true;

}

// Puts the center of the ball in imageX and imageY
bool BallDetector::findBall2(std::vector<Blob>& blobs, int& imageX, int& imageY) {
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
      return false;
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
    
    //std::cout << "row: " << row << " col: " << col << " width: " << width << " height: " << height << "\n";
    //std::cout << " xPadding: " << xPadding << " yPadding: " << yPadding << " width: " << width << " height: " << height;
    //std::cout << " col: " << col << " row: " << row << " xf: " << col + width << " yf: " << row + height << "\n";
    
    //frame = color::rawToMatSubset(vblocks_.image->getImgTop(), iparams_, row, col, width, height, 1, 1);
    grayFrame = color::rawToMatGraySubset(vblocks_.image->getImgTop(), iparams_, row, col, width, height, 1, 1);
    
    std::vector<cv::Vec3f> circles;
  
    if (!grayFrame.data) {
      return false;
    }
    
    // Check to see if getting rid of this speeds anything up
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(3, 3), 0, 0);

    cv::HoughCircles(grayFrame, circles, CV_HOUGH_GRADIENT, 20, grayFrame.rows/8, 10, 10, 0, 0);
    
    // Now turn circles into a vector of floats
    // Populates a v with circles.size() elements, each a vector with 3 floats
    std::vector<std::vector<float>> v(circles.size(), std::vector<float>(3));
  
    for (size_t i = 0; i < circles.size(); ++i) {
      const cv::Vec3f& c = circles[i];
      v[i][0] = c[0] + col; // if downsampled
      v[i][1] = c[1] + row; // if downsampled
      v[i][2] = c[2];
  
      std::cout << "Circle " << i << " x: " << v[i][0] << " y: " << v[i][1] << " r: " << v[i][2] << "\n";
    }
  }
  
  imageX = imageY = 0;
  int total = 0;
  float x_mean, y_mean;
  x_mean = y_mean = 0.0;

  //std::cout << "iparams_.width: " << iparams_.width << " iparams_.height: " << iparams_.height << "\n";
  
  // Process from left to right
  for(int x = 0; x < iparams_.width; x++) {
    // Process from top to bottom
    for(int y = 0; y < iparams_.height; y++) {
      // Retrieve the segmented color of the pixel at (x,y)
      auto c = getSegImg()[y * iparams_.width + x];
      if(c == c_ORANGE){
        total++;
        x_mean = x_mean + x;
        y_mean = y_mean + y;
      }
    }
  }
  
  if(total <= 20){
    return false;
  }
  x_mean = x_mean/total;
  y_mean = y_mean/total;
  imageX = (int)x_mean;
  imageY = (int)y_mean;

  std::cout << "ImageX: " << imageX << " ImageY: " << imageY << "\n";
  
  // TO DO: Add heuristics to return false if ball detection fails
  return true;
}

void BallDetector::findBall() {
  if(camera_ == Camera::BOTTOM) return;
  static map<WorldObjectType,int> heights = {
    { WO_BALL, 33 },
  };
  static map<WorldObjectType,vector<int>> balls = {
    { WO_BALL, { 24, 15, 74, 83} }
  };
  auto fid = vblocks_.frame_info->frame_id;
  if(fid >= 6150) return;
  for(auto ball : balls) {
    auto& object = vblocks_.world_object->objects_[ball.first];
    auto box = ball.second;
    object.imageCenterX = (box[0] + box[2]) / 2;
    object.imageCenterY = (box[1] + box[3]) / 2;
    auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[ball.first]);
    object.visionDistance = cmatrix_.groundDistance(position);
    object.visionBearing = cmatrix_.bearing(position);
    object.seen = true;
    object.fromTopCamera = camera_ == Camera::TOP;
    tlog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(ball.first), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }
}
