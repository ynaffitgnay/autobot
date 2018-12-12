#include "DStarLite.h"
#include <iostream>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/PlanningBlock.h>
#include <common/GridCell.h>

DStarLite::DStarLite(TextLogger*& tlogger)
  : tlogger_(tlogger), cells_(nullptr), k_(0), lastReplanIdx(0), nodeExpansions(0),
    initialized(false), goalIdx_(-1), endPlanIdx_(-1), path_(nullptr) {
}
  
void DStarLite::init(std::vector<GridCell>& wavefront, int goal, int start, std::vector<int>* path) {
  cells_ = &wavefront;
  k_ = 0;
  path_ = path;
  map_.clear();
  lastReplanIdx = 0;
  goalIdx_ = goal;
  endPlanIdx_ = start;
  
  // Make a 2D array out of PathNodes
  if (!buildPathGrid()) {
    std::cout << "EMPTY PATH GRID!" << std::endl;
  }

  S_ = &(map_.at(goalIdx_));

  U_ = DSLPQueue();

  S_->rhs = 0;

  // Calculate the key for the start node
  S_->key = calcKey(*S_);

  // Insert the start into the pqueue
  U_.push(S_);

  ++nodeExpansions;
}

// runDSL with no replanning
int DStarLite::runDSL() {
  std::vector<PathNode>::iterator mapIt;

  if (endPlanIdx_ == -1) {
    std::cout << "Trying to run regular D Star Lite on coverage path!!" << std::endl;
  }

  computeShortestPath(map_.at(endPlanIdx_));

  int pathSize = generatePath(0);
  //printGrid();
  return pathSize;

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
    U_.remove(uPtr);
    u.key = calcKey(u);
    U_.push(uPtr);
  } else if (u.g != u.rhs && !U_.contains(uPtr)) {
    u.key = calcKey(u);
    U_.push(uPtr);
    ++nodeExpansions;
  } else {
    U_.remove(uPtr);
  }
}

// Generates cost for each cell in the grid
void DStarLite::computeShortestPath(PathNode& curr) {
  // Don't compute for occupied cells
  if (curr.cell.occupied) {
    //std::cout << "Cell is occupied." << std::endl;
    return;
  }
  PathNode* u = nullptr;
  
  // Calculate the key for the current node
  while (U_.top()->key < calcKey(curr) || curr.rhs > curr.g) {
    //++nodeExpansions;
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
      getNeighbors(*u, preds);

      for (predIt = preds.begin(); predIt != preds.end(); predIt++) {
        int c = getTransitionCost(*u, **predIt);
        int t = safeAdd(c, u->g);
        (*predIt)->rhs = std::min((*predIt)->rhs, t);
        updateVertex(**predIt);
      }
      
    } else {
      int g_old = u->g;
      u->g = INT_MAX;

      std::vector<PathNode*> preds;
      std::vector<PathNode*>::iterator predIt;
      getNeighbors(*u, preds);
      preds.push_back(u);

      for (predIt = preds.begin(); predIt != preds.end(); predIt++) {
        int t = safeAdd(getTransitionCost(*u, **predIt), g_old);
        if ((*predIt)->rhs == t) {
          if (*predIt != S_) {  // if the PathNode being pointed to isn't the goal
            int minrhs = INT_MAX;
            std::vector<PathNode*> succs;
            std::vector<PathNode*>::iterator succIt;

            getNeighbors(**predIt, succs);

            for (succIt = succs.begin(); succIt != succs.end(); succIt++) {
              int t2 = safeAdd(getTransitionCost(**predIt, **succIt), (*succIt)->g); 
              if (t2 < minrhs) {
                minrhs = t2;
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

void DStarLite::getNeighbors(PathNode& curr, vector<PathNode*>& neighbors) {
  int s_r = curr.cell.r;
  int s_c = curr.cell.c;

  if (s_r - 1 >= 0 && !map_.at(PathNode::getIdx(s_r - 1, s_c)).cell.occupied) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r - 1, s_c))));
  }
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT && !map_.at(PathNode::getIdx(s_r + 1, s_c)).cell.occupied) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r + 1, s_c))));
  }
  
  // Check left side
  if (s_c - 1 >= 0 && !map_.at(PathNode::getIdx(s_r, s_c - 1)).cell.occupied) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r, s_c - 1))));
  }
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH && !map_.at(PathNode::getIdx(s_r, s_c + 1)).cell.occupied) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r, s_c + 1))));
  }
}

void DStarLite::getUnplannedNeighbors(PathNode& curr, vector<PathNode*>& neighbors) {
  int s_r = curr.cell.r;
  int s_c = curr.cell.c;
  // Check above
  if (s_r - 1 >= 0 && !map_.at(PathNode::getIdx(s_r - 1, s_c)).cell.occupied && !map_.at(PathNode::getIdx(s_r - 1, s_c)).planned) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r - 1, s_c))));
  }
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT && !map_.at(PathNode::getIdx(s_r + 1, s_c)).cell.occupied && !map_.at(PathNode::getIdx(s_r + 1, s_c)).planned) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r + 1, s_c))));
  }
  
  // Check left side
  if (s_c - 1 >= 0 && !map_.at(PathNode::getIdx(s_r, s_c - 1)).cell.occupied && !map_.at(PathNode::getIdx(s_r, s_c - 1)).planned) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r, s_c - 1))));
  }
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH && !map_.at(PathNode::getIdx(s_r, s_c + 1)).cell.occupied && !map_.at(PathNode::getIdx(s_r, s_c + 1)).planned) {
    neighbors.push_back(&(map_.at(PathNode::getIdx(s_r, s_c + 1))));
  }
}


int DStarLite::getTransitionCost(PathNode& s, PathNode& p) {
  if (p.cell.occupied) return INT_MAX;

  if (s.cell.r == p.cell.r && s.cell.c == p.cell.c) return 0;

  // can add more costs for different types of movement here (e.g., based on turning?)
  return 1;
}

// Generate path from idx startIdx
int DStarLite::generatePath(int startIdx) {
  int pathIdx = startIdx;
  int currCellIdx = endPlanIdx_;
  
  // Plan order of poses
  map_.at(endPlanIdx_).planned = true;
  map_.at(endPlanIdx_).pathorder = pathIdx;
  path_->at(pathIdx++) = currCellIdx;   // Add the start pose as the first pose in the plan
  
  int numPlanned = 1;                        // start and end cell
  while(currCellIdx != goalIdx_)
  {
    std::vector<PathNode*> unplanned;
    getUnplannedNeighbors(map_.at(currCellIdx), unplanned);
    if(unplanned.size() > 0) // We can only move to a neighbor if it is valid and unplanned
    {
      int minValue = unplanned[0]->getValue();
      int minIndex = 0;                       
      if(unplanned.size() > 1) 
      {
        for(int i = 1; i < unplanned.size(); i++)
        {
          if (unplanned[i]->idx == goalIdx_) {
            minValue = unplanned[i]->getValue();
            minIndex = i;
            break;
          }
          
          if(unplanned[i]->getValue() < minValue)
          {
            minValue = unplanned[i]->getValue();
            minIndex = i;
          }
        }
      }
      int lastIndex = currCellIdx;
      currCellIdx = unplanned.at(minIndex)->idx;

      map_.at(currCellIdx).pathorder = pathIdx;
      map_.at(currCellIdx).planned = true;
      path_->at(pathIdx++) = currCellIdx;
      numPlanned++;
    }
    else // if all valid neighbors have been visited, we are stuck
    {
      std::cout << "Stuck: no neighbors are unplanned." << std::endl;
      //printPath();
      return -1;
    }
  }
  
  //printPath();
  //printf("Path size: %d\n", numPlanned);
  printf("Node expansions in vanilla DSL: %d\n", nodeExpansions);
  
  return numPlanned;
}

bool DStarLite::buildPathGrid() {
  // TODO: add checks for robustness
  if (cells_->size() <= 0) {
    std::cout << "Wavefront grid is empty" << std::endl;
    return false;
  }

  std::vector<GridCell>::iterator gridIt;
  
  for (gridIt = cells_->begin(); gridIt != cells_->end(); gridIt++) {
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

void DStarLite::printGrid() {
  int index;

  // Colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  GridCell* cell = nullptr;

  std::cout << "     ";
  for (int col = 0; col < GRID_WIDTH; ++col) {
    if (col < 10) {
      std::cout << "  " << col << "  ";
    } else {
      std::cout << " " << col << "  ";
    }
  }
  std::cout << std::endl;
  for (int row = 0; row < GRID_HEIGHT; ++row) {
    std::cout << "  g |";
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col;
      if (map_.at(index).cell.occupied) {
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      } else {
        if (map_.at(index).g < 10 && map_.at(index).g >= 0) {
          std::cout << "|  "<< map_.at(index).g <<"|";
        } else if (map_.at(index).g == INT_MAX) {  
          std::cout << "|"<< "inf" <<"|";
        } else if (map_.at(index).g > 99) {
          std::cout << "|"<< map_.at(index).g <<"|"; 
        } else {
          std::cout << "| "<< map_.at(index).g <<"|";
        }
      }
    }
    std::cout << std::endl;

    std::cout << "rhs |";
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col;
      if (map_.at(index).cell.occupied) {
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      } else {
        if (map_.at(index).rhs < 10 && map_.at(index).rhs >= 0) {
          std::cout << "|  "<< map_.at(index).rhs <<"|";
        } else if (map_.at(index).rhs == INT_MAX) {
          std::cout << "|" << "inf" <<"|";
        } else if (map_.at(index).rhs > 99) {
          std::cout << "|" << map_.at(index).rhs <<"|"; 
        } else {
          std::cout << "| "<< map_.at(index).rhs <<"|";
        }
      }
    }
    std::cout << "  " << row << std::endl;

    std::cout << "  h |";
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col; 
      if (map_.at(index).cell.occupied) {
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      } else {
        if (map_.at(index).cell.cost < 10 && map_.at(index).cell.cost >= 0) {
          std::cout << "|  "<< map_.at(index).cell.cost <<"|"; 
        } else if (map_.at(index).cell.cost == INT_MAX) {
          std::cout << "|"<< "inf" <<"|";
        } else if (map_.at(index).cell.cost > 99) {
          std::cout << "|"<< map_.at(index).cell.cost <<"|"; 
        } else {
          std::cout << "| "<< map_.at(index).cell.cost <<"|";
        }
      }
    }
    std::cout << std::endl;

    std::cout << "    |";
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col;
      std::cout << " --- ";
    }
    std::cout << std::endl;
  }
  std::cout << "     ";
  for (int col = 0; col < GRID_WIDTH; ++col) {
    if (col < 10) {
      std::cout << "  " << col << "  ";
    } else {
      std::cout << " " << col << "  ";
    }
  }
  std::cout << std::endl;
}


void DStarLite::printPath() {
  int index;

  // Colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  GridCell* cell = nullptr;

  std::cout << "     ";
  for (int col = 0; col < GRID_WIDTH; ++col) {
    if (col < 10) {
      std::cout << "  " << col << "  ";
    } else {
      std::cout << " " << col << "  ";
    }
  }
  
  std::cout << std::endl;
  for (int row = 0; row < GRID_HEIGHT; ++row) {
    if (row < 10) {
      std::cout << "  " << row << " |";
    } else {
      std::cout << " " << row << " |";
    }
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col;
      if (map_.at(index).cell.occupied) {
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      } else {
        if (map_.at(index).pathorder < 10 && map_.at(index).pathorder >= 0) {
          std::cout << "|  "<< map_.at(index).pathorder <<"|";
        } else if (map_.at(index).pathorder == INT_MAX) {  
          std::cout << "|"<< "inf" <<"|";
        } else if (map_.at(index).pathorder > 99) {
          std::cout << "|"<< map_.at(index).pathorder <<"|"; 
        } else {
          std::cout << "| "<< map_.at(index).pathorder <<"|";
        }
      }
    }
    std::cout << "  " << row << std::endl;
  }
  
  std::cout << "     ";
  for (int col = 0; col < GRID_WIDTH; ++col) {
    if (col < 10) {
      std::cout << "  " << col << "  ";
    } else {
      std::cout << " " << col << "  ";
    }
  }
  std::cout << std::endl;
}
