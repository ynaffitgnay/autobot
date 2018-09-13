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
  std::vector<Blob> colorBlobs[5];
  std::vector<VisionPointAlt> runs;
  constructRuns(runs);
  mergeRegions(runs, colorBlobs);
}

void Classifier::constructRuns(std::vector<VisionPointAlt>& runs) {
  // Process from top to bottom
  uint16_t x,y;
  VisionPointAlt* lUndef = nullptr;
  VisionPointAlt* lGreen = nullptr;
  VisionPointAlt* lWhite = nullptr;
  VisionPointAlt* lOrange = nullptr;
  VisionPointAlt* lPink = nullptr;
  VisionPointAlt* lBlue = nullptr;
  VisionPointAlt* lYellow = nullptr;
  VisionPointAlt* lRobo = nullptr;
  
  for (y = 0; y < iparams_.height; y++) {
    auto runColor = segImg_[y * iparams_.width];
    // Create a new VisionPointAlt for the first run in this line
    VisionPointAlt run = VisionPointAlt(0, y, runColor);

    // Set the nextVP for the previous run of this color to this run
    switch (runColor) {
      case c_UNDEFINED:
        if (lUndef) {
          lUndef->nextVP = &run;
        }
        lUndef = &run;
        break;
      case c_FIELD_GREEN:
        if (lGreen) {
          lGreen->nextVP = &run;
        }
        lGreen = &run;
        break;
      case c_WHITE:
        if (lWhite) {
          lWhite->nextVP = &run;
        }
        lWhite = &run;
        break;
      case c_ORANGE:
        if (lOrange) {
          lOrange->nextVP = &run;
        }
        lOrange = &run;
        break;
      case c_PINK:
        if (lPink) {
          lPink->nextVP = &run;
        }
        lPink = &run;
        break;
      case c_BLUE:
        if (lBlue) {
          lBlue->nextVP = &run;
        }
        lBlue = &run;
        break;
      case c_YELLOW:
        if (lYellow) {
          lYellow->nextVP = &run;
        }
        lYellow = &run;
        break;
      case c_ROBOT_WHITE:
        if (lRobo) {
          lRobo->nextVP = &run;
        }
        lRobo = &run;
        break;
      default:
        cout << "This shouldn't happen\n";
    }
    
    // Process from left to right
    for (x = 0; x < iparams_.width; x++) {
      auto pixelColor = segImg_[y * iparams_.width + x];

      if (pixelColor == runColor) {
        // fill the visionpoint alt (increase the x)
        run.xf = x;
        run.dx++;
      } else {
        // put the current run into the vector and create a new run with this color
        runs.push_back(run);

        ////TODO: get rid of this
        //if (run.dx > 10) {
        //  if (runColor == c_UNDEFINED) {
        //    //std::cout << "run_length: " << run.dx << " runColor: UNDEFINED\n";
        //  } else if (runColor == c_FIELD_GREEN) {
        //    //std::cout << "run_length: " << run.dx << " runColor: GREEN\n";
        //  } else if (runColor == c_WHITE) {           
        //    //std::cout << "run_length: " << run.dx << " runColor: WHITE\n";
        //  } else if (runColor == c_ORANGE) {          
        //    std::cout << "run_length: " << run.dx << " runColor: ORANGE\n";
        //  } else if (runColor == c_PINK) {            
        //    //std::cout << "run_length: " << run.dx << " runColor: PINK\n";
        //  } else if (runColor == c_BLUE) {            
        //    //std::cout << "run_length: " << run.dx << " runColor: BLUE\n";
        //  } else if (runColor == c_YELLOW) {          
        //    //std::cout << "run_length: " << run.dx << " runColor: YELLOW\n";
        //  } else if (runColor == c_ROBOT_WHITE) {     
        //    //std::cout << "run_length: " << run.dx << " runColor: ROBOT\n";
        //  } else {
        //    std::cout << "what?";
        //  }
        //}
        
        runColor = pixelColor;
        run = VisionPointAlt(x, y, runColor);
        switch (runColor) {
          case c_UNDEFINED:
            if (lUndef) {
              lUndef->nextVP = &run;
            }
            lUndef = &run;
            break;
          case c_FIELD_GREEN:
            if (lGreen) {
              lGreen->nextVP = &run;
            }
            lGreen = &run;
            break;
          case c_WHITE:
            if (lWhite) {
              lWhite->nextVP = &run;
            }
            lWhite = &run;
            break;
          case c_ORANGE:
            if (lOrange) {
              lOrange->nextVP = &run;
            }
            lOrange = &run;
            break;
          case c_PINK:
            if (lPink) {
              lPink->nextVP = &run;
            }
            lPink = &run;
            break;
          case c_BLUE:
            if (lBlue) {
              lBlue->nextVP = &run;
            }
            lBlue = &run;
            break;
          case c_YELLOW:
            if (lYellow) {
              lYellow->nextVP = &run;
            }
            lYellow = &run;
            break;
          case c_ROBOT_WHITE:
            if (lRobo) {
              lRobo->nextVP = &run;
            }
            lRobo = &run;
            break;
          default:
            cout << "This shouldn't happen\n";
        }
      }
    }
    // Finish the last run in this row
    run.xf = x;
    run.dx++;

    //TODO: get rid of this
    //if (run.dx > 10) {
    //  if (runColor == c_UNDEFINED) {
    //    //std::cout << "run_length: " << run.dx << " runColor: UNDEFINED\n";
    //  } else if (runColor == c_FIELD_GREEN) {
    //    //std::cout << "run_length: " << run.dx << " runColor: GREEN\n";
    //  } else if (runColor == c_WHITE) {           
    //    //std::cout << "run_length: " << run.dx << " runColor: WHITE\n";
    //  } else if (runColor == c_ORANGE) {          
    //    std::cout << "run_length: " << run.dx << " runColor: ORANGE\n";
    //  } else if (runColor == c_PINK) {            
    //    //std::cout << "run_length: " << run.dx << " runColor: PINK\n";
    //  } else if (runColor == c_BLUE) {            
    //    //std::cout << "run_length: " << run.dx << " runColor: BLUE\n";
    //  } else if (runColor == c_YELLOW) {          
    //    //std::cout << "run_length: " << run.dx << " runColor: YELLOW\n";
    //  } else if (runColor == c_ROBOT_WHITE) {     
    //    //std::cout << "run_length: " << run.dx << " runColor: ROBOT\n";
    //  } else {
    //    std::cout << "what?";
    //  }
    //}

    //put the current run into the vector
    runs.push_back(run);
  }

  //TODO:
  //std::cout << "Total runs: " << runs.size() << "\n";
}

void Classifier::mergeRegions(std::vector<VisionPointAlt>& runs, std::vector<Blob> (&blobs)[5]) {
  int regionsMerged = 0;

  // Don't do undefined, green, or white blobs
  std::vector<Blob> orange;
  std::vector<Blob> pink;
  std::vector<Blob> blue;
  std::vector<Blob> yellow;
  std::vector<Blob> robo;
  std::vector<Blob> * bloblist = nullptr;

  bool merged;
  uint32_t runIdx = 0;
  unsigned char runColor;

  for (auto run : runs) {
    runColor = run.color;
    switch(runColor) {
      case c_ORANGE:
        bloblist = &orange;
        break;
      case c_PINK:
        bloblist = &pink;
        break;
      case c_BLUE:
        bloblist = &blue;
        break;
      case c_YELLOW:
        bloblist = &yellow;
        break;
      case c_ROBOT_WHITE:
        bloblist = &robo;
        break;
      default:
        // if it's one of the other colors, don't process this run
        continue;
    }

    merged = false;
    
    // Check to see if this region fits in a blob
    std::cout << "NEW BLOBLIST!\n";
    for (auto blob : *bloblist)
    {
      if (merged) {
        continue;
      }
      //TODO
      if (run.dx > 2) {
        std::cout << "run.xi: " << run.xi << "run.xf: " << run.xf << "run.yi: " << run.yi << "blob.yf: " << blob.yf << "\n";
      }
      // make sure the rows are adjacent
      if (run.yi == blob.yf + 1)
      {
        if (run.xi >= blob.xi && run.xi <= blob.xf) {
          // Add this region to the blob
          merged = true;
          ++regionsMerged;
          blob.yf = run.yf;
          if (run.xf > blob.xf) {
            blob.xf = run.xf;
          }
          blob.lpIndex[blob.lpCount] = runIdx;
          blob.lpCount++;
          blob.widthEnd = run.dx;

          //TODO: calculate avgX and avgY

        } else if (run.xf >= blob.xi && run.xf <= blob.xf) {
          merged = true;
          ++regionsMerged;
          blob.yf = run.yf;
          if (run.xi < blob.xi) {
            blob.xi = run.xi;
          }
          blob.lpIndex[blob.lpCount] = runIdx;
          blob.lpCount++;
          blob.widthEnd = run.dx;

          //TODO: calculate avgX and avgY
          
        } else if (run.xi <= blob.xi && run.xf >= blob.xf) {
          merged = true;
          ++regionsMerged;
          blob.yf = run.yf;
          blob.xi = run.xi;
          blob.xf = run.xf;
          blob.lpIndex[blob.lpCount] = runIdx;
          blob.lpCount++;
          blob.widthEnd = run.dx;
        }
      }
    }

    // make a new blob for this run
    if (!merged) {
      Blob newBlob = Blob(runColor, run.xi, run.xf, run.dx, run.yi, run.yf, run.dy, run.dx);
      newBlob.lpIndex[0] = runIdx;
      newBlob.lpCount++;
      bloblist->push_back(newBlob);

      std::cout << "Bloblist size: " << bloblist->size() << "\n";

      //TODO: calculate avgX and avgY
    }
    
    ++runIdx;
    // TODO
    int totalBlobs = orange.size() + pink.size() + blue.size() + yellow.size() + robo.size();
    std::cout << "Regions merged: " << regionsMerged << " Runs:" << runs.size() << " Blobs:" << totalBlobs << "\n";
  }

  blobs[0] = orange;
  blobs[1] = pink;
  blobs[2] = blue;
  blobs[3] = yellow;
  blobs[4] = robo;
  
}

void Classifier::mergeRegions(std::vector<Blob> (&blobs)[5]) {
  int regionsMerged = 0;
  
  // Recursively call mergeRegions on the new merged regions
  if (regionsMerged != 0) {
    mergeRegions(blobs);
  }
  // only want to merge for ball, beacons, and goal
}

void Classifier::getStepSize(int& h, int& v) const {
    h = 1 << 2;
    v = 1 << 1;
}
