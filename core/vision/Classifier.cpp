#include "Classifier.h"
#include <vision/Logging.h>
#include <iostream>
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
  std::vector<VisionPointAlt> runs;
  constructRuns(runs);
}

void Classifier::constructRuns(std::vector<VisionPointAlt>& runs) {
  // Process from top to bottom
  uint16_t x,y;
  for (y = 0; y < iparams_.height; y++) {
    auto run_color = segImg_[y * iparams_.width];
    // Create a new VisionPointAlt for the first run in this line
    VisionPointAlt run = VisionPointAlt(0, y, run_color);
    
    // Process from left to right
    for (x = 0; x < iparams_.width; x++) {
      auto pixel_color = segImg_[y * iparams_.width + x];

      if (pixel_color == run_color) {
        // fill the visionpoint alt (increase the x)
        run.xf = x;
        run.dx++;
      } else {
        // put the current run into the vector and create a new run with this color
        runs.push_back(run);

        //TODO: get rid of this
        if (run_color == c_UNDEFINED) {
          std::cout << "run_length: " << run.dx << " run_color: UNDEFINED\n";
        } else if (run_color == c_FIELD_GREEN) {
          std::cout << "run_length: " << run.dx << " run_color: GREEN\n";
        } else if (run_color == c_WHITE) {           
          std::cout << "run_length: " << run.dx << " run_color: WHITE\n";
        } else if (run_color == c_ORANGE) {          
          std::cout << "run_length: " << run.dx << " run_color: ORANGE\n";
        } else if (run_color == c_PINK) {            
          std::cout << "run_length: " << run.dx << " run_color: PINK\n";
        } else if (run_color == c_BLUE) {            
          std::cout << "run_length: " << run.dx << " run_color: BLUE\n";
        } else if (run_color == c_YELLOW) {          
          std::cout << "run_length: " << run.dx << " run_color: YELLOW\n";
        } else if (run_color == c_ROBOT_WHITE) {     
          std::cout << "run_length: " << run.dx << " run_color: ROBOT\n";
        } else {
          std::cout << "what?";
        }
        
        run_color = pixel_color;
        run = VisionPointAlt(x, y, run_color);
      }
        
    }
    // Finish the last run in this row
    run.xf = x;
    run.dx++;

    //put the current run into the vector
    runs.push_back(run);
  }
  
}

void Classifier::getStepSize(int& h, int& v) const {
    h = 1 << 2;
    v = 1 << 1;
}
