#include <vision/LineDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

LineDetector::LineDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void LineDetector::findPenaltyLine(std::vector<Blob>& blobs) {
  if(camera_ == Camera::TOP) return;
  // std::cout << std::endl;
  // std::cout << "Frame ID: " << vblocks_.frame_info->frame_id << std::endl;
  // std::cout << std::endl;
  // printf("1\n");
  std::vector<Blob> line_cand;
  std::vector<std::vector<WorldObject> > line_list;
  for (auto& blob : blobs) {
    if (blob.invalid) {
      // printf("Blob invalid\n");
    } else {
      // printf("Blob valid\n");
    }
    if(blob.color == c_WHITE) {
      line_cand.push_back(blob);
    }
  }
  // printf("2\n");
  if (line_cand.size() > 0) {
    // printf("3\n");
    detBestLine(line_cand);
  }
}

void LineDetector::detBestLine(std::vector<Blob>& line_cand) {
  Blob bestLine;
  std::vector<Blob> line_options;
  bestLine.invalid = true;
  // printf("4\n");
  for (auto& line : line_cand) {
    // printf("5\n");
    int size = line.total;
    if (size > 1200) {
      // printf("6\n");
      line_options.push_back(line);
    } else {
      // printf("Blob at [%d,%d] is totally not the line with aspect ratio: %4.2f:%4.2f => %4.5f and %d pixels\n",line.avgX,line.avgY,height,width,height/width, size);
        // printf("7\n");
    }
  }
  
  double minAspectRatio = 99.0;
  for (auto& line_opt : line_options) {
    // printf("8\n");
    double width = line_opt.xf - line_opt.xi;  // width
    double height = line_opt.yf-line_opt.yi; //height
    double density = line_opt.pixelDensity;
    double aspectRatio = height/width;
    if (aspectRatio < minAspectRatio) {
      // printf("9\n");
        minAspectRatio = aspectRatio;
        bestLine = line_opt;
        bestLine.invalid = false;
    }   
  }

  if (bestLine.invalid) {
      // printf("10\n");
    return;
  }

  // printf("Blob at [%d,%d] is totally the line with aspect ratio: %d:%d => %4.5f and %d pixels\n",bestLine.avgX,bestLine.avgY,bestLine.yf-bestLine.yi,bestLine.xf-bestLine.xi,(double)(bestLine.yf-bestLine.yi)/(double)(bestLine.xf-bestLine.xi),bestLine.total);
  auto& object = vblocks_.world_object->objects_[WO_OWN_PENALTY];
  auto pos1 = cmatrix_.getWorldPosition(bestLine.xi,bestLine.avgY,0);
  auto pos2 = cmatrix_.getWorldPosition(bestLine.xf,bestLine.avgY,0);
  float lh_dist = cmatrix_.groundDistance(pos1);
  float lh_bear = cmatrix_.bearing(pos1);
  float rh_dist = cmatrix_.groundDistance(pos2);
  float rh_bear = cmatrix_.bearing(pos2);
  float xlh_rel = lh_dist*cos(lh_bear);
  float ylh_rel = lh_dist*sin(lh_bear);
  float xrh_rel = rh_dist*cos(rh_bear);
  float yrh_rel = rh_dist*sin(rh_bear);

  Point2D lh_point(xlh_rel,ylh_rel);
  Point2D rh_point(xrh_rel,yrh_rel);
  LineSegment line_seg(lh_point,rh_point);
  object.lineLoc = line_seg;
  object.seen = true;
  object.fromTopCamera = camera_ == Camera::BOTTOM;
  
}

