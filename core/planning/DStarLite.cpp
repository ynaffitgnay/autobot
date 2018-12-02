#include "DStarLite.h"
#include <iostream>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>
#include <common/GridCell.h>

DStarLite::DStarLite(MemoryCache& cache, TextLogger*& tlogger, Point2D S)
  : cache_(cache), tlogger_(tlogger), startCoords_(S), wf_(nullptr), k_(0),
    movementCostSinceReplan(0), initialized(false) {
}
  
void DStarLite::init(Grid& wavefront) {
  wf_ = &wavefront;
  k_ = 0;
  map_.clear();
  
  // Make a 2D array out of PathNodes
  if (!buildPathGrid()) {
    std::cout << "EMPTY PATH GRID!" << std::endl;
  }

  // Set pointer to start node
  //std::cout << "gridRow: " << getGridRow(startCoords_.y) << " gridCol: " << getGridCol(startCoords_.x) << std:: endl;
  //std::cout << "Result of getIdx: " << PathNode::getIdx(getGridRow(startCoords_.y), getGridCol(startCoords_.x)) << std::endl;
  S_ = &(map_.at(PathNode::getIdx(getGridRow(startCoords_.y), getGridCol(startCoords_.x))));
  std::cout << "Debug 2\n";

  U_ = DSLPQueue();

  S_->rhs = 0;

  // Calculate the key for the start node
  S_->key = calcKey(*S_);

  // Insert the start into the pqueue
  U_.push(S_);
}

void DStarLite::runDSL() {
  PathNode* S_last = nullptr;
  PathNode* S_curr = nullptr;
  std::vector<PathNode>::iterator mapIt;

  // here we're going to need to update the DSL from a certain coordinate
  // TODO: update this function to use the robot's current location when replanning
  if (!initialized) {
    if (map_.size() <= 0) {
      std::cout << "EMPTY MAP IN RUNDSL?!\n" << std::endl;
      return;
    }
    
    // Initialize costs for each node to S
    for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
      computeShortestPath(*mapIt);
      // Print map after each iteration??
    }
    generatePath();
    initialized = true;
  }


  //TODO:
  // if "changed":

  for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
    if (!mapIt->changed) continue;

    computeShortestPath(*mapIt);
  }
  
}

DSLKey DStarLite::calcKey(PathNode& successor) {
  int k_1 = 0;
  int k_2 = std::min(successor.g, successor.rhs);
  // Check for integer overflow
  k_1 = safeAdd(k_2, successor.cell.cost);
  k_1 = safeAdd(k_1, k_);
  
  return DSLKey(k_1, k_2);
}

void DStarLite::updateVertex(PathNode& u) {
  PathNode* uPtr = &u;
  if (u.g != u.rhs && U_.contains(uPtr)) {
    std::cout << "Vertex is in the pq!" << std::endl;
    U_.remove(uPtr);
    u.key = calcKey(u);
    U_.push(uPtr);
  } else if (u.g != u.rhs && !U_.contains(uPtr)) {
    u.key = calcKey(u);
    U_.push(uPtr);
  } else {
    U_.remove(uPtr);
  }
}

void DStarLite::computeShortestPath(PathNode& curr) {
  PathNode* u = nullptr;
  
  // Calculate the key for the current node
  while (U_.top()->key < calcKey(curr) || curr.rhs > curr.g) {
    u = U_.top();
    DSLKey k_new = calcKey(*u);

    if (U_.top()->key < k_new) {
      U_.pop();
      // Update priority of u
      u->key = k_new;
      U_.push(u);
    } else if (u->g > u->rhs) {
      U_.pop();
      u->g = u->rhs;
      
      std::vector<PathNode*> preds;
      std::vector<PathNode*>::iterator predIt;
      getPreds(*u, preds);

      for (predIt = preds.begin(); predIt != preds.end(); predIt++) {
        // Ignore occupied nodes
        //if (predIt->occupied) continue;
        int c = getTransitionCost(*u, **predIt);
        //int t = (c > INT_MAX - u->g) ? INT_MAX : c + u->g;
        int t = safeAdd(c, u->g);
        (*predIt)->rhs = std::min((*predIt)->rhs, t);
        updateVertex(**predIt);
      }
      
    } else {
      int g_old = u->g;
      u->g = INT_MAX;

      std::vector<PathNode*> preds;
      std::vector<PathNode*>::iterator predIt;
      getPreds(*u, preds);
      preds.push_back(u);

      for (predIt = preds.begin(); predIt != preds.end(); predIt++) {
        int t = safeAdd(getTransitionCost(*u, **predIt), g_old);
        if ((*predIt)->rhs == t) {
          if (*predIt != S_) {  // if the PathNode being pointed to isn't the goal
            int minrhs = INT_MAX;
            std::vector<PathNode*> succs;
            std::vector<PathNode*>::iterator succIt;

            getSuccs(**predIt, succs);

            for (succIt = succs.begin(); succIt != succs.end(); succIt++) {
              int t2 = safeAdd(getTransitionCost(**predIt, **succIt), (*succIt)->g); 
              if (t2 < minrhs) {
                minrhs = t2;//safeAdd(getTransitionCost(**predIt, **succIt), (*succIt)->g);
              }
            }
            (*predIt)->rhs = minrhs;
          }
        }
        updateVertex(**predIt);
      }
    }
  }
}

void DStarLite::getPreds(PathNode& successor, vector<PathNode*>& preds) {
  int s_r = successor.cell.r;
  int s_c = successor.cell.c;
  // Check above
  if (s_r - 1 >= 0) preds.push_back(&(map_.at(PathNode::getIdx(s_r - 1, s_c))));
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT) preds.push_back(&(map_.at(PathNode::getIdx(s_r + 1, s_c))));
  
  // Check left side
  if (s_c - 1 >= 0) preds.push_back(&(map_.at(PathNode::getIdx(s_r, s_c - 1))));
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH) preds.push_back(&(map_.at(PathNode::getIdx(s_r, s_c + 1))));
}

void DStarLite::getSuccs(PathNode& predecessor, vector<PathNode*>& succs) {
  int s_r = predecessor.cell.r;
  int s_c = predecessor.cell.c;
  // Check above
  if (s_r - 1 >= 0) succs.push_back(&(map_.at(PathNode::getIdx(s_r - 1, s_c))));
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT) succs.push_back(&(map_.at(PathNode::getIdx(s_r + 1, s_c))));
  
  // Check left side
  if (s_c - 1 >= 0) succs.push_back(&(map_.at(PathNode::getIdx(s_r, s_c - 1))));
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH) succs.push_back(&(map_.at(PathNode::getIdx(s_r, s_c + 1))));
}


int DStarLite::getTransitionCost(PathNode& s, PathNode& p) {
  if (p.cell.occupied) return INT_MAX;

  if (s.cell.r == p.cell.r && s.cell.c == p.cell.c) return 0;

  // can add more costs for different types of movement here (e.g., based on turning?)
  return 1;
}

void DStarLite::generatePath() {
}

// Maybe do these steps in generateGrid
bool DStarLite::buildPathGrid() {
  // TODO: add checks for robustness
  if (wf_->cells.size() <= 0) {
    std::cout << "Wavefront grid is empty" << std::endl;
    return false;
  }

  std::vector<GridCell>::iterator gridIt;
  
  for (gridIt = wf_->cells.begin(); gridIt != wf_->cells.end(); gridIt++) {
    //PathNode cell = PathNode(gridIt->r, gridIt->c, gridIt->cost, gridIt->occupied);
    PathNode cell = PathNode(*(gridIt));
    map_.push_back(cell);
  }

  if (map_.size() != GRID_SIZE) {
    std::cout << "map_.size(): " << map_.size() << " GRID_SIZE: " << GRID_SIZE << std::endl;
    std::cout << "GRID_SIZE ALL WRONG WHEN BUILDING PATH_GRID???" << std::endl;
  }
  
  return true;
}

int DStarLite::safeAdd(int q1, int q2) {
  return ((q1 > INT_MAX - q2) ? INT_MAX : q1 + q2);
}
