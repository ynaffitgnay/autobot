#include "Classifier.h"
#include <vision/Logging.h>
#include <iostream>
#include <fstream>
#include <inttypes.h>

using namespace cv;

Classifier::Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera) :
    vblocks_(vblocks), vparams_(vparams), iparams_(iparams), camera_(camera), initialized_(false) {
  segImg_ = new unsigned char[iparams.size];
  segImgLocal_ = segImg_;
  setImagePointers();
}

Classifier::~Classifier() {
  delete [] segImgLocal_;
}

bool Classifier::setImagePointers() {
  if(vblocks_.image == NULL) {
    printf("No image block loaded! Classification failed.\n");
    return false;
  }
  if(vblocks_.robot_vision == NULL) {
    printf("No vision block loaded! Classification failed.\n");
    return false;
  }
  bool imageLoaded = vblocks_.image->isLoaded();
  if(camera_ == Camera::TOP) {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgTop(segImg_);
      img_ = vblocks_.image->getImgTop();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->isLoaded()) {
      segImg_ = vblocks_.robot_vision->getSegImgTop();
    }
    #endif
  }
  else {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgBottom(segImg_);
      img_ = vblocks_.image->getImgBottom();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->isLoaded()) {
      segImg_ = vblocks_.robot_vision->getSegImgBottom();
    }
    #endif
  }
  if(!initialized_) {
    #ifdef TOOL
    if(imageLoaded)
    #endif
    memset(segImg_, c_UNDEFINED, sizeof(unsigned char) * iparams_.size);
    initialized_ = true;
  }
  return true;
}

bool Classifier::classifyImage(unsigned char *colorTable) {
  if(!setImagePointers()) return false;
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  classifyImage(area, colorTable);
  return true;
}

void Classifier::classifyImage(const FocusArea& area, unsigned char* colorTable) {
  bool imageLoaded = vblocks_.image->isLoaded();
  if(!imageLoaded) {
    tlog(20, "Classifying with no raw image");
  }
  colorTable_ = colorTable;
  int vstep = 1 << 1;
  int hstep = 1 << 2;
  for (int y = area.y1; y <= area.y2; y += vstep) {
    for(int x = area.x1; x <= area.x2; x += hstep) {
      auto c = ColorTableMethods::xy2color(img_, colorTable, x, y, iparams_.width);
      segImg_[iparams_.width * y + x] = c;
    }
  }
}

void Classifier::makeBlobs(std::vector<Blob>& blobs) {
  std::vector<VisionPointAlt> runs;
  std::vector<std::vector<VisionPointAlt*>> parents;
  std::vector<VisionPointAlt*>::iterator parentIt;
  unsigned char c;
  uint16_t xi, xf, dx, yi, yf, dy, widthStart, widthEnd, avgX, avgY, total;
  Point2D start, end;
  float meanX, meanY, pixelRatio, pixelDensity;

  // Get the run-length encoding for the frame and compress paths
  constructRuns(runs);
  mergeRuns(runs);
  makeParentLists(runs,parents);
  
  // Make blobs from runs
  for (int i = 0; i < parents.size(); ++i)
  {
    // Allow smaller orange blobs (for small, far away balls)
    if ((parents.at(i).front()->color != c_ORANGE && parents.at(i).size() < 5) || parents.at(i).size() < 3)
    {
      continue;
    }
    
    total = 0;
    xi = parents.at(i).front()->xi;
    xf = parents.at(i).front()->xf;
    dx = parents.at(i).front()->dx;
    widthStart = parents.at(i).front()->dx;
    c = parents.at(i).front()->color;
    yi = parents.at(i).front()->yi;
    yf = parents.at(i).front()->yf;
    dy = parents.at(i).front()->dy;
    widthEnd = parents.at(i).front()->dx;
    avgX = 0;
    avgY = 0;
    meanX = 0.0;
    meanY = 0.0;
    uint16_t lpcout = 0;
    for(parentIt = parents.at(i).begin(); parentIt != parents.at(i).end(); parentIt++){
      if((*parentIt)->xi < xi) xi = (*parentIt)->xi;
      if((*parentIt)->xf > xf) xf = (*parentIt)->xf;
      if((*parentIt)->yi < yi) widthStart = (*parentIt)->dx;
      if((*parentIt)->yf > yf) widthEnd = (*parentIt)->dx;
      if((*parentIt)->yi < yi) yi = (*parentIt)->yi;
      if((*parentIt)->yf > yf) yf = (*parentIt)->yf;
      total += (*parentIt)->dx;
      meanX += (0.5*((*parentIt)->xi + (*parentIt)->xf) * ((*parentIt)->dx));
      meanY += (0.5*((*parentIt)->yi + (*parentIt)->yf) * ((*parentIt)->dx));
      lpcout += 1;
    }
    dy = yf-yi;
    dx = xf-xi;
    
    avgX = (uint16_t) (meanX/total);
    avgY = (uint16_t) (meanY/total);
    pixelRatio = (float)(xf - xi) / (float)(yf - yi);
    pixelDensity = (float)total / (float)((xf - xi) * (yf - yi));
    Blob blob = Blob(c, xi, xf, dx, yi, yf, dy, widthStart, widthEnd, avgX, avgY, total, pixelRatio, pixelDensity,lpcout);
    blobs.push_back(blob);
  }
  
}

void Classifier::makeParentLists(std::vector<VisionPointAlt>& runs, std::vector<std::vector<VisionPointAlt*>>& parents) {
  if (runs.size() <= 0) return;
  
  std::vector<VisionPointAlt>::iterator iter = runs.begin();
  std::vector<std::vector<VisionPointAlt*>>::iterator parentIt;
  bool found = false;
  for(iter; iter!=runs.end(); iter++){
    if(iter->parent){
      found = false;
      for(parentIt = parents.begin(); parentIt != parents.end(); parentIt++){
        if (parentIt->front() == iter->parent)
        {
          parentIt->push_back(&(*iter));
          found = true;
          break;
        }
      }
      if (!found)
      {
        std::vector<VisionPointAlt*> newParent;
        newParent.push_back(iter->parent);
        parents.push_back(newParent);
      }
    }
  }
}

void Classifier::constructRuns(std::vector<VisionPointAlt>& runs) {
  uint16_t x,y;
  for (y = 0; y < iparams_.height; y+=2) {
    auto run_color = segImg_[y * iparams_.width];
    int undef_count = 0;
    
    // Create a new VisionPointAlt for the first run in this line
    VisionPointAlt run = VisionPointAlt(0, y, run_color);
    // Process from left to right
    for (x = 0; x < iparams_.width; x+=4) {
      auto pixel_color = segImg_[y * iparams_.width + x];
      
      if (pixel_color == run_color) {
        // fill the visionpoint alt (increase the x)
        run.xf = x;
        run.dx += 4;
      } else {
        // put the current run into the vector and create a new run with this color
        runs.push_back(run);
        run_color = pixel_color;
        run = VisionPointAlt(x, y, run_color);
      }
    }
    // Finish the last run in this row
    run.xf = x;
    run.dx += 4;

    //put the current run into the vector
    runs.push_back(run);
  }
  
}

void Classifier::mergeRuns(std::vector<VisionPointAlt>& runs) {
  int counter = 0;
  int vpa_num = runs.size();
  int cRow = 0;
  int pvRow;

  if (runs.size() <= 0) return;

  // Iterators to a list of runs in the row above the current run
  std::vector<VisionPointAlt>::iterator iter = runs.begin();
  std::vector<VisionPointAlt>::iterator row_begin = iter;
  std::vector<VisionPointAlt>::iterator row_end;
  
  bool first = true;

  // Cycle through the runs 
  // First pass
  for(iter; iter !=runs.end(); iter++) {
    iter->parent = &(*iter);
    iter->rank = vpa_num-counter;
    counter++;
    if (iter->yi != cRow) {  // When we are no longer on the same row, we must be in the next
      first = false;
      row_end = iter;
      cRow = iter->yi;
    }
    if(!first){
      checkAdj(*iter, row_begin, row_end);
    }
  }

  // Second pass
  iter = runs.begin();
  for(iter; iter !=runs.end(); iter++) {
    if(camera_ == Camera::BOTTOM) {
      if (iter->color != c_UNDEFINED && iter->color != c_FIELD_GREEN) {
        // Just running through the clean up the parent pointers for disjoint parent chains
        iter->parent = findParent(*iter);
      }
    } else if(camera_ == Camera::TOP) {
      if (iter->color != c_UNDEFINED && iter->color != c_FIELD_GREEN) {
        // Just running through the clean up the parent pointers for disjoint parent chains
        iter->parent = findParent(*iter);
      }
    }
  }
}

void Classifier::checkAdj(VisionPointAlt& node, std::vector<VisionPointAlt>::iterator row_begin, std::vector<VisionPointAlt>::iterator row_end) {
  if(camera_ == Camera::BOTTOM) {
    if (node.color == c_UNDEFINED || node.color == c_FIELD_GREEN) {
      node.parent = nullptr;
      return;
    }
  } else if(camera_ == Camera::TOP) {
    if (node.color == c_UNDEFINED || node.color == c_FIELD_GREEN) {
      node.parent = nullptr;
      return;
    }
  }
  for(row_begin; row_begin != row_end; row_begin++) {
    if (row_begin->color == node.color)
    {
    if (((node.xi >= row_begin->xi) && node.xi <= row_begin->xf) || ((node.xf >= row_begin->xi) && node.xf <= row_begin->xf) || ((node.xf >= row_begin->xf) && node.xi <= row_begin->xi))
      {
        if (node.yi - (row_begin->yf) < 5) {
          unionByRank(node, *row_begin);
        }
      }
    }
  }
}

VisionPointAlt * Classifier::findParent(VisionPointAlt& node) {
  VisionPointAlt *nodePtr = &node;
  if (node.parent != nodePtr) {
    node.parent = findParent(*(node.parent)); //Recursive loop to find parent
  }
  return node.parent;
}

void Classifier::unionByRank(VisionPointAlt& a, VisionPointAlt& b) {
  VisionPointAlt *rootA = findParent(a);
  VisionPointAlt *rootB = findParent(b);
  VisionPointAlt *temp;
  if (rootA == rootB) {
    //Already merged, return
    return;
  }
  if (rootA->rank < rootB->rank) {
    // Swap parents
    temp = rootA;
    rootA = rootB;
    rootB = temp;
    temp = NULL;
  }

  rootB->parent = rootA;  // Put the parent back
  if (rootA->rank == rootB->rank) {
    rootA->rank++;  // Increment rank
  }
}

void Classifier::getStepSize(int& h, int& v) const {
    h = 1 << 2;
    v = 1 << 1;
}
