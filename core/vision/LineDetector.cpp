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
  std::vector<Blob> line_cand;
  std::vector<std::vector<WorldObject> > line_list;
  for (auto& blob : blobs) {
    if(blob.color == c_WHITE) {
      line_cand.push_back(blob);
    }
  }

  if (line_cand.size() > 0) {
    detBestLine(line_cand);
  }

  
}

void LineDetector::detBestLine(std::vector<Blob> line_cand) {
  Blob bestLine;
  std::vector<Blob> line_options;
  bestLine.invalid = true;
  for (auto& line : line_cand) {
    int size = line.total;
    if (size > 1200) {
      line_options.push_back(line);
    } else {
      // printf("Blob at [%d,%d] is totally not the line with aspect ratio: %4.2f:%4.2f => %4.5f and %d pixels\n",line.avgX,line.avgY,height,width,height/width, size);
    }
  }
  
  double minAspectRatio = 99.0;
  for (auto& line_opt : line_options) {
    double width = line_opt.xf - line_opt.xi;  // width
    double height = line_opt.yf-line_opt.yi; //height
    double density = line_opt.pixelDensity;
    double aspectRatio = height/width;
    if (aspectRatio < minAspectRatio) {
        minAspectRatio = aspectRatio;
        bestLine = line_opt;
    }   
  }

  if (bestLine.invalid) {
    return;
  }

  printf("Blob at [%d,%d] is totally the line with aspect ratio: %d:%d => %4.5f and %d pixels\n",bestLine.avgX,bestLine.avgY,bestLine.yf-bestLine.yi,bestLine.xf-bestLine.xi,(double)(bestLine.yf-bestLine.yi)/(double)(bestLine.xf-bestLine.xi),bestLine.total);
  auto& object = vblocks_.world_object->objects_[WO_OWN_PENALTY];
  Point2D lh_point(bestLine.xi,bestLine.avgY);
  Point2D rh_point(bestLine.xf,bestLine.avgY);
  LineSegment line_seg(lh_point,rh_point);
  object.lineLoc = line_seg;
  object.seen = true;
  object.fromTopCamera = camera_ == Camera::BOTTOM;
  
}

