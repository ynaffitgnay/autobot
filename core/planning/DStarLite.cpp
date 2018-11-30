#include "DStarLite.h"
#include <iostream>

DStarLite::DStarLite(MemoryCache& cache, TextLogger*& tlogger)
  : cache_(cache), tlogger_(tlogger), wf_(nullptr), k_(0) {
}
  
void DStarLite::init(const Grid& wavefront) {
  wf_ = &wavefront;
  k_ = 0;
  map_.clear();
  
  // Make a 2D array out of PathNodes
  buildPathGrid();

  U_ = priority_queue<PathNode>();
  
    
}

void DStarLite::runDSL() {
}

int DStarLite::calcKey(PathNode successor) {

  //TODO: don't return 0
  return 0;
}

void DStarLite::updateVertex(PathNode u) {
}

void DStarLite::computeShortestPath() {
}


// Maybe do these steps in generateGrid
bool DStarLite::buildPathGrid() {
  // TODO: add checks for robustness
  if (wf_->cells.size() <= 0) {
    std::cout << "Wavefront grid is empty" << std::endl;
    return false;
  }
  
  for (int r = 0; r < GRID_HEIGHT; ++r) {
    vector<PathNode> row;
    if (wf_->cells.at(r).size() <= 0) {
      std::cout << "Unexpected empty row" << std::endl;
      continue;
    }
    for (int c = 0; c < GRID_WIDTH; ++c) {
      PathNode cell = PathNode(r,c,wf_->cells.at(r).at(c).cost);
      row.push_back(cell);
    }
    map_.push_back(row);
  }
  return true;
}

