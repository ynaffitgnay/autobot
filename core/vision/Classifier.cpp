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

void Classifier::getBlobs(std::vector<Blob>& blobs) {
  std::vector<Blob> colorBlobs[5];
  std::vector<VisionPointAlt> runs;
  constructRuns(runs);
  mergeRuns(runs, blobs);
}

void Classifier::constructRuns(std::vector<VisionPointAlt>& runs) {
  uint16_t x,y;
  for (y = 0; y < iparams_.height; y+=2) {
    auto run_color = segImg_[y * iparams_.width];
    int undef_count = 0;
    
    // Create a new VisionPointAlt for the first run in this line
    VisionPointAlt run = VisionPointAlt(0, y / 2, run_color);
    // Process from left to right
    for (x = 0; x < iparams_.width; x+=4) {
      auto pixel_color = segImg_[y * iparams_.width + x];
            
      if (pixel_color == run_color) {
        // fill the visionpoint alt (increase the x)
        run.xf = x / 4;
        run.dx++;
      } else {
        // put the current run into the vector and create a new run with this color
        runs.push_back(run);
        run_color = pixel_color;
        run = VisionPointAlt(x / 4, y / 2, run_color);
      }
    }
    // Finish the last run in this row
    run.xf = x / 4;
    run.dx++;

    //put the current run into the vector
    runs.push_back(run);
  }
  
}

void Classifier::mergeRuns(std::vector<VisionPointAlt>& runs, std::vector<Blob>& blobs) {
  // TODO: get rid of this
  std::cout << "Merge " << runs.size() << " runs" << std::endl; 
  int counter = 0;
  int vpa_num = runs.size();
  int cRow = 0;
  int pvRow;
  
  std::vector<VisionPointAlt>::iterator iter = runs.begin();
  std::vector<VisionPointAlt>::iterator row_begin = iter;
  std::vector<VisionPointAlt>::iterator row_end;
  
  bool first = true;
  for(iter; iter !=runs.end(); iter++) {
    iter->parent = &(*iter);
    iter->rank = vpa_num-counter;
    counter++;
    if (iter->yi != cRow) {
      first = false;
      row_end = iter;
      cRow = iter->yi;
    }
    if(!first){
      checkAdj(*iter, row_begin, row_end);
    }
  }

  std::cout << "Hi we mergedruns\n";
}

void Classifier::checkAdj(VisionPointAlt& node, std::vector<VisionPointAlt>::iterator row_begin, std::vector<VisionPointAlt>::iterator row_end) {
  if (node.color == c_UNDEFINED || node.color == c_FIELD_GREEN || node.color == c_WHITE) {
    return;
  }
  for(row_begin; row_begin != row_end; row_begin++) {
    if (row_begin->color == node.color)
    {
    if (((node.xi >= row_begin->xi) && node.xi <= row_begin->xf) || ((node.xf >= row_begin->xi) && node.xf <= row_begin->xf) || ((node.xf >= row_begin->xf) && node.xi <= row_begin->xi))
      {
        unionByRank(node, *row_begin);
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
  if ((*rootA).rank < (*rootB).rank) {
    // Swap parents
    temp = rootA;
    rootA = rootB;
    rootB = temp;
    temp = NULL;
  }

  (*rootB).parent = rootA;  // Put the parent back
  if ((*rootA).rank == (*rootB).rank) {
    (*rootA).rank++;  // Increment rank
  }
}

void Classifier::getStepSize(int& h, int& v) const {
    h = 1 << 2;
    v = 1 << 1;
}