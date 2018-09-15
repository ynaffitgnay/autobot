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
  //mergeRegions(runs, colorBlobs);
}

void Classifier::constructRuns(std::vector<VisionPointAlt>& runs) {
  //todo: get rid of this shit
  //ofstream myfile;
  //myfile.open ("/home/parallels/tifflocal/newbettercrzylog.txt");

  //string homedir = std::getenv("HOME");
  //std::cout << "homedir: " << homedir << endl;
  
  
  //TODO: REPLACE ALL STRUCTS WITH "NEW" STRUCTS
  // Process from top to bottom
  uint16_t x,y;
  for (y = 0; y < iparams_.height; y+=2) {
    auto run_color = segImg_[y * iparams_.width];
    int undef_count = 0;
    
    // Create a new VisionPointAlt for the first run in this line
    VisionPointAlt run = VisionPointAlt(0, y / 2, run_color);
    // Process from left to right
    for (x = 0; x < iparams_.width; x+=4) {
      // TODO: get rid of this
      auto pixel_color = segImg_[y * iparams_.width + x];
            
      //myfile << (uint32_t)((uint8_t)pixel_color) << " ";      

      if (pixel_color == run_color) {
        // fill the visionpoint alt (increase the x)
        run.xf = x / 4;
        run.dx++;
      } else {
        // put the current run into the vector and create a new run with this color
        runs.push_back(run);

        //TODO: get rid of this
        //std::cout << "run_length: " << run.dx << "run_color:" << run.color << "\n";
        
        run_color = pixel_color;
        run = VisionPointAlt(x / 4, y / 2, run_color);
      }
        
    }
    // Finish the last run in this row
    run.xf = x / 4;
    run.dx++;

    //TODO: get rid of this:
    //myfile << "\n";

    //put the current run into the vector
    runs.push_back(run);
  }
  //TODO
  //myfile.close();
  
}

void Classifier::mergeRuns(std::vector<VisionPointAlt>& runs, std::vector<Blob>& blobs) {
  std::cout << "Merge " << runs.size() << " runs" << std::endl; 
  int counter = 0;
  // No default constructor for VisionPointAlt, so you have to have a dummy for the struct
  
  // Just a dummy as the initial. Might need to review this
  int vpa_num = runs.size();
  int cRow = 0;
  int pvRow;
  // std::deque<VisionPointAlt*> adjRowCurr;
  // std::deque<VisionPointAlt*> adjRowPrev;
  std::vector<VisionPointAlt>::iterator iter = runs.begin();
  std::vector<VisionPointAlt>::iterator row_begin = iter;
  std::vector<VisionPointAlt>::iterator row_end;
  // std::copy(iter,row_begin);
  bool first = true;
  for(iter; iter !=runs.end(); iter++) {
    iter->parent = &(*iter);
    // std::cout << "VPA #" << counter << ":" << std::endl;
    //std::cout << "Run length: " << (*iter).dy << "    Run color: " << (int)(*iter).color << std::endl;
    

    // node->parent = &node;
    // std::cout << "1 Node Rank: " << iter->rank << std::endl;
    iter->rank = vpa_num-counter;
    // std::cout << "2 Node Rank: " << iter->rank << std::endl;
    counter++;
    if (iter->yi != cRow) {
      first = false;
      row_end = iter;
      // Moved to next row
      // std::cout << "New Row" << std::endl; 
      cRow = iter->yi;

      // adjRowPrev.clear();
      // adjRowPrev.insert(std::end(adjRowPrev), std::begin(adjRowCurr), std::end(adjRowCurr));
      // adjRowCurr.clear();

    }
    // std::cout << "PosCheck X: "  << node->data.xi << " PosCheck Y: " << node->data.yi << std::endl;
    // adjRowCurr.push_back(&(*iter));
    if(!first){
      // printf("%p\t %p\n", (void *) &(*row_begin),(void *) &(*row_end));
      checkAdj(*iter, row_begin, row_end);
    }
  }

  std::cout << "Hi we mergedruns\n";

  //TODO: fill blobs


}

void Classifier::checkAdj(VisionPointAlt& node, std::vector<VisionPointAlt>::iterator row_begin, std::vector<VisionPointAlt>::iterator row_end) {
  // printf("%p\t %p\n", (void *) &(*row_begin),(void *) &(*row_end));
// void Classifier::checkAdj(VisionPointAlt& node, std::deque<VisionPointAlt*> adjRowPrev) {
  //not filled in yet
  // std::cout << "adjRowPrev size: " << adjRowPrev.size() << std::endl;
  // for(std::deque<VisionPointAlt*>::iterator iter = adjRowPrev.begin(); iter !=adjRowPrev.end(); iter++) {
  for(row_begin; row_begin != row_end; row_begin++) {
    // std::cout << "Iter Rank: "  << (*iter)->rank << " PosCheck X: "  << (*iter)->xi << " PosCheck Y: " << (*iter)->yi << std::endl;
    if (row_begin->color == node.color)
    {
    if (((node.xi >= row_begin->xi) && node.xi <= row_begin->xf) || ((node.xf >= row_begin->xi) && node.xf <= row_begin->xf) || ((node.xf >= row_begin->xf) && node.xi <= row_begin->xi))
      {
        // std::cout << "Starting Union" << std::endl;
        // std::cout << "3 Node Rank: " << node.rank << std::endl;
        // printf("%p\t %p\n", (void *) &node,(void *) node.parent);
        unionByRank(node, *row_begin);
        //node.parent unionByRank(iter-->parent;
      }
    }
  }
}

VisionPointAlt * Classifier::findParent(VisionPointAlt& node) {
  // std::cout << "Finding Parent" << std::endl; 
  // std::cout << "Node Rank: " << node.parent->rank << std::endl;
  VisionPointAlt *nodePtr = &node;

  // if (node.parent != nodePtr) {
  //   node.parent = findParent(*(node.parent)); //Recursive loop to find parent
  //   nodePtr = nodePtr->parent;
  // }
  while(nodePtr->parent != nodePtr){
    // std::cout << "Iter Rank: "  << nodePtr->rank << " PosCheck X: "  << nodePtr->xi << " PosCheck Y: " << nodePtr->yi << std::endl;
    // printf("nodePtr = %p\t nodePtr->parent = %p\n", (void *) nodePtr,(void *) (*nodePtr).parent);
    nodePtr = nodePtr->parent;
  }

  node.parent = nodePtr;
  return node.parent;
}

void Classifier::setParent(VisionPointAlt& node, VisionPointAlt* newParent){
  //TODO: reset all upstream parents to new parent
}

void Classifier::unionByRank(VisionPointAlt& a, VisionPointAlt& b) {
  // std::cout << "Running Union" << std::endl; 
  // printf("&a = %p\t a.parent = %p\n", (void *) &a,(void *) a.parent);
  // std::cout << "Node Rank: " << a.parent->rank << "Node Parent rank: " << b.parent->rank << std::endl;
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


void Classifier::mergeRegions(std::vector<VisionPointAlt>& runs, std::vector<Blob> (&blobs)[5]) { }
//  // Merge regions from runs
//  int regionsMerged = 0;
//
//  std::vector<Blob> * bloblist = nullptr;
//
//  bool merged;
//  uint32_t runIdx = 0;
//  unsigned char runColor;
//
//  // Don't do undefined, green, or white blobs
//  for (auto run : runs) {
//    runColor = run.color;
//    switch(runColor) {
//      case c_ORANGE:
//        bloblist = &(blobs[0]);
//        break;
//      case c_PINK:
//        bloblist = &(blobs[1]);
//        break;
//      case c_BLUE:
//        bloblist = &(blobs[2]);
//        break;
//      case c_YELLOW:
//        bloblist = &(blobs[3]);
//        break;
//      case c_ROBOT_WHITE:
//        bloblist = &(blobs[4]);
//        break;
//      default:
//        // if it's one of the other colors, don't process this run
//        continue;
//    }
//
//    merged = false;
//    
//    // Check to see if this region fits in a blob
//    for (auto blob : *bloblist)
//    {
//      if (merged) {
//        continue;
//      }
//
//      // NOTE: if this coarse blob-merging does not give us good enough granularity,
//      // can look through list of runs associated with each blob to ensure that there
//      // is true overlap instead of disparate pieces
//      
//      // make sure the rows are adjacent
//      if (run.yi == blob.yf + 1)
//      {
//        if (run.xi >= blob.xi && run.xi <= blob.xf) {
//          // Add this region to the blob
//          merged = true;
//          ++regionsMerged;
//          blob.yf = run.yf;
//          if (run.xf > blob.xf) {
//            blob.xf = run.xf;
//          }
//          blob.lpIndex[blob.lpCount] = runIdx;
//          blob.lpCount++;
//          blob.widthEnd = run.dx;
//
//          //TODO: calculate avgX and avgY
//
//        } else if (run.xf >= blob.xi && run.xf <= blob.xf) {
//          merged = true;
//          ++regionsMerged;
//          blob.yf = run.yf;
//          if (run.xi < blob.xi) {
//            blob.xi = run.xi;
//          }
//          blob.lpIndex[blob.lpCount] = runIdx;
//          blob.lpCount++;
//          blob.widthEnd = run.dx;
//
//          //TODO: calculate avgX and avgY
//          
//        } else if (run.xi <= blob.xi && run.xf >= blob.xf) {
//          merged = true;
//          ++regionsMerged;
//          blob.yf = run.yf;
//          blob.xi = run.xi;
//          blob.xf = run.xf;
//          blob.lpIndex[blob.lpCount] = runIdx;
//          blob.lpCount++;
//          blob.widthEnd = run.dx;
//        }
//      }
//    }
//    
//    // make a new blob for this run
//    if (!merged) {
//      Blob newBlob = Blob(runColor, run.xi, run.xf, run.dx, run.yi, run.yf, run.dy, run.dx);
//      newBlob.lpIndex[0] = runIdx;
//      newBlob.lpCount++;
//      bloblist->push_back(newBlob);
//
//      std::cout << "Bloblist size: " << bloblist->size() << "\n";
//
//      //TODO: calculate avgX and avgY
//    }
//    
//    ++runIdx;
//    // TODO
//    int totalBlobs = blobs[0].size() + blobs[1].size() + blobs[2].size() + blobs[3].size() + blobs[4].size(); 
//    std::cout << "Regions merged: " << regionsMerged << " Runs:" << runs.size() << " Blobs:" << totalBlobs << "\n";
//  }
//
//  // Now recursively merge these blobs
//  mergeRegionsFromBlobs(runs, blobs);  
//}
//

void Classifier::mergeRegionsFromBlobs(std::vector<VisionPointAlt>& runs, std::vector<Blob> (&blobs)[5]) { }
//  // only want to merge for ball, beacons, and goal
//  int regionsMerged = 0;
//  int colorIdx; // 0 = ORANGE, 1 = PINK, 2 = BLUE, 3 = YELLOW, 4 = ROBOT_WHITE 
//  
//
//  // Check if blobs overlap
//  // When you merge a blob, update the values of the first blob, and then mark the second blob as invalid
//  // Then pop all invalid blobs from the blob vector?
//  for (colorIdx = 0; colorIdx < 5; colorIdx++)
//  {
//    std::vector<Blob>::iterator blobIt;
//    // int blobIdx = 0;
//    // If this blob marked invalid, erase it and move iterator, else check the rest of the blobs
//    for (blobIt = blobs[colorIdx].begin(); blobIt != blobs[colorIdx].end(); blobIt++); 
//    {
//      // Delete blobs that are marked invalid and advance the iterator
//      
//    }
//  }
//
//  // Recursively call mergeRegions on the new merged regions
//  if (regionsMerged != 0) {
//    mergeRegionsFromBlobs(runs, blobs);
//  }
//}

void Classifier::getStepSize(int& h, int& v) const {
    h = 1 << 2;
    v = 1 << 1;
}


