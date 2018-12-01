#include "DStarLite.h"
#include <iostream>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>

DStarLite::DStarLite(MemoryCache& cache, TextLogger*& tlogger, Point2D S)
  : cache_(cache), tlogger_(tlogger), startCoords_(S), wf_(nullptr), k_(0) {
}
  
void DStarLite::init(const Grid& wavefront) {
  //int S_r;
  //int S_c;
  
  wf_ = &wavefront;
  k_ = 0;
  map_.clear();
  
  // Make a 2D array out of PathNodes
  if (!buildPathGrid()) {
    std::cout << "EMPTY PATH GRID!" << std::endl;
  }

  // Set pointer to start node
  S_ = &(map_.at(getGridRow(startCoords_.y)).at(getGridCol(startCoords_.x)));

  U_ = DSLPQueue();//priority_queue<PathNode*, vector<PathNode*>, PNCmp>();

  //TODO: change this to accept input
  //auto& robot = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  //
  //S_r = getGridRow(robot.loc.y);
  //S_c = getGridCol(robot.loc.x);
  //map_.at(S_r).at(S_c).rhs = 0;

  S_->rhs = 0;

  // Calculate the key for the start node
  //calcKey(map_.at(S_r_).at(S_c_));
  S_->key = calcKey(*S_);
  
  //map_.at(S_r_).at(S_c_).initialized = true;
  S_->initialized = true;

  // Insert the start into the pqueue
  //U_.push(&(map_.at(S_r_).at(S_c_)));
  U_.push(S_);
}

//void DStarLite::runCCDSL() {
//  // Run DSL on 
//}

void DStarLite::runDSL() {
  // here we're going to need to update the DSL from a certain coordinate
  // TODO: update this function to use the robot's current location when replanning

  PathNode* S_last = nullptr;
  PathNode* S_curr = nullptr;

  // TODO: do i have to init here??
  
  // Initialize costs for each node to S
  for (int r = 0; r < GRID_HEIGHT; ++r) {
    if (map_.at(r).size() <= 0) {
      std::cout << "Unexpected empty row" << std::endl;
      continue;
    }
    for (int c = 0; c < GRID_WIDTH; ++c) {
      //if ((r == S_->r && c == S_->c) || map_.at(r).at(c).initialized) {
      if (map_.at(r).at(c).initialized) {
        continue;
      }
      computeShortestPath(map_.at(r).at(c));
      // TODO: print here
    }
  }
}

DSLKey DStarLite::calcKey(PathNode& successor) {
  int k_1 = 0;
  int k_2 = std::min(successor.g, successor.rhs);
  // Check for integer overflow
  //k_1 = (k_2 > INT_MAX - successor.h) ? INT_MAX : k_2 + successor.h;
  k_1 = safeAdd(k_2, successor.h);
  //k_1 = (k_1 > INT_MAX - k_) ? INT_MAX : k_1 + k_;
  k_1 = safeAdd(k_1, k_);
  
  return DSLKey(k_1, k_2);
}

void DStarLite::updateVertex(PathNode& u) {
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
  int s_r = successor.r;
  int s_c = successor.c;
  // Check above
  if (s_r - 1 >= 0) preds.push_back(&(map_.at(s_r - 1).at(s_c)));
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT) preds.push_back(&(map_.at(s_r + 1).at(s_c)));
  
  // Check left side
  if (s_c - 1 >= 0) preds.push_back(&(map_.at(s_r).at(s_c - 1)));
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH) preds.push_back(&(map_.at(s_r).at(s_c + 1)));
}

void DStarLite::getSuccs(PathNode& predecessor, vector<PathNode*>& succs) {
  int s_r = predecessor.r;
  int s_c = predecessor.c;
  // Check above
  if (s_r - 1 >= 0) succs.push_back(&(map_.at(s_r - 1).at(s_c)));
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT) succs.push_back(&(map_.at(s_r + 1).at(s_c)));
  
  // Check left side
  if (s_c - 1 >= 0) succs.push_back(&(map_.at(s_r).at(s_c - 1)));
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH) succs.push_back(&(map_.at(s_r).at(s_c + 1)));
}


int DStarLite::getTransitionCost(PathNode& s, PathNode& p) {
  if (p.occupied) return INT_MAX;

  if (s.r == p.r && s.c == p.c) return 0;

  // can add more costs for different types of movement here (e.g., based on turning?)
  return 1;
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

int DStarLite::safeAdd(int q1, int q2) {
  return ((q1 > INT_MAX - q2) ? INT_MAX : q1 + q2);
}
