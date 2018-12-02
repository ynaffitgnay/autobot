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
  //std::cout << "Debug 2\n";

  U_ = DSLPQueue();

  S_->rhs = 0;

  // Calculate the key for the start node
  S_->key = calcKey(*S_);

  // Insert the start into the pqueue
  U_.push(S_);

  // Perform the initial cost calculations for each cell
  std::cout << "not initialized yet!\n";
  //printGrid();
  if (map_.size() <= 0) {
    std::cout << "EMPTY MAP IN RUNDSL?!\n" << std::endl;
    return;
  }

  std::vector<PathNode>::iterator mapIt;

  std::cout << "About to start computing shortest paths\n";
  // Initialize costs for each node to S

  int calcNode = 0;
  for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
    if (S_ == &(*mapIt)) {
      std::cout << "Skipping start point" << std::endl;
      continue;
    }
    std::cout << "Processing node (" << mapIt->cell.r << ", " << mapIt->cell.c << "), node " << calcNode++ << " / " << map_.size() << std::endl;
    computeShortestPath(*mapIt);
    // Print map after each iteration??
    //printGrid();
    if (mapIt->g < mapIt->cell.cost) {
      std::cout << "HEURISTICS FOR CELL AT (" << mapIt->cell.r << ", " << mapIt->cell.c << ") INCONSISTENT\n\n\n\n";
      return;
    }
    std::cout << "\n\n";
  }
  std::cout << "About to generate path!\n";
  generatePath();
  initialized = true;
}

void DStarLite::runDSL() {
  PathNode* S_last = nullptr;
  PathNode* S_curr = nullptr;
  std::vector<PathNode>::iterator mapIt;

  //std::cout << "in runDSL\n";

  // here we're going to need to update the DSL from a certain coordinate
  // TODO: update this function to use the robot's current location when replanning
  //if (!initialized) {
  //  
  //}

  //std::cout << "already initalized\n";


  //TODO:
  // if "changed":

  //TODO: make sure that no cells left on the path are occupied

  // TODO: BFS from start node. the way that you're currently computing
  // costs is insane!
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

// Generates cost for each cell in the grid
void DStarLite::computeShortestPath(PathNode& curr) {
  // Don't compute for occupied cells
  if (curr.cell.occupied) {
    std::cout << "Cell is occupied." << std::endl;
    return;
  }
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
  if (s_r - 1 >= 0 && !map_.at(PathNode::getIdx(s_r - 1, s_c)).cell.occupied) {
    preds.push_back(&(map_.at(PathNode::getIdx(s_r - 1, s_c))));
  }
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT && !map_.at(PathNode::getIdx(s_r + 1, s_c)).cell.occupied) {
    preds.push_back(&(map_.at(PathNode::getIdx(s_r + 1, s_c))));
  }
  
  // Check left side
  if (s_c - 1 >= 0 && !map_.at(PathNode::getIdx(s_r, s_c - 1)).cell.occupied) {
    preds.push_back(&(map_.at(PathNode::getIdx(s_r, s_c - 1))));
  }
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH && !map_.at(PathNode::getIdx(s_r, s_c + 1)).cell.occupied) {
    preds.push_back(&(map_.at(PathNode::getIdx(s_r, s_c + 1))));
  }
}

void DStarLite::getSuccs(PathNode& predecessor, vector<PathNode*>& succs) {
  int s_r = predecessor.cell.r;
  int s_c = predecessor.cell.c;
  
  //// Check above
  //if (s_r - 1 >= 0) succs.push_back(&(map_.at(PathNode::getIdx(s_r - 1, s_c))));
  //
  //// Check below
  //if (s_r + 1 < GRID_HEIGHT) succs.push_back(&(map_.at(PathNode::getIdx(s_r + 1, s_c))));
  //
  //// Check left side
  //if (s_c - 1 >= 0) succs.push_back(&(map_.at(PathNode::getIdx(s_r, s_c - 1))));
  //
  //// Check right side
  //if (s_c + 1 < GRID_WIDTH) succs.push_back(&(map_.at(PathNode::getIdx(s_r, s_c + 1))));

  if (s_r - 1 >= 0 && !map_.at(PathNode::getIdx(s_r - 1, s_c)).cell.occupied) {
    succs.push_back(&(map_.at(PathNode::getIdx(s_r - 1, s_c))));
  }
  
  // Check below
  if (s_r + 1 < GRID_HEIGHT && !map_.at(PathNode::getIdx(s_r + 1, s_c)).cell.occupied) {
    succs.push_back(&(map_.at(PathNode::getIdx(s_r + 1, s_c))));
  }
  
  // Check left side
  if (s_c - 1 >= 0 && !map_.at(PathNode::getIdx(s_r, s_c - 1)).cell.occupied) {
    succs.push_back(&(map_.at(PathNode::getIdx(s_r, s_c - 1))));
  }
  
  // Check right side
  if (s_c + 1 < GRID_WIDTH && !map_.at(PathNode::getIdx(s_r, s_c + 1)).cell.occupied) {
    succs.push_back(&(map_.at(PathNode::getIdx(s_r, s_c + 1))));
  }
}



int DStarLite::getTransitionCost(PathNode& s, PathNode& p) {
  if (p.cell.occupied) return INT_MAX;

  if (s.cell.r == p.cell.r && s.cell.c == p.cell.c) return 0;

  // can add more costs for different types of movement here (e.g., based on turning?)
  return 5;
}

void DStarLite::generatePath() {
  // make sure that no cell on the path is occupied
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
    if (gridIt->occupied) {
      std::cout << "Occupied cost = " << gridIt->cost << std::endl;
      //gridIt->cost = INT_MAX;
      //std::cout << "New cost = " << (*(gridIt)).cost << std::endl;
    }
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
  //int grid_cols = num_cols_;
  //int grid_rows = num_rows_;
  int index;

  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  GridCell* cell = nullptr;

  //std::cout << "---------------------g & h-Values-----------------------------------" << std::endl;
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
      index = row * GRID_WIDTH + col; // Finding the appropriate cell in vectorized form
      //if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
      if (map_.at(index).cell.occupied) {
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      } else {
        //if(waveCells[index].getValue()<10 && waveCells[index].getValue()>=0)
        if (map_.at(index).g < 10 && map_.at(index).g >= 0) {
          std::cout << "|  "<< map_.at(index).g <<"|"; 
        } else if (map_.at(index).g == INT_MAX) {
          std::cout << "|"<< "inf" <<"|";
        } else {
          std::cout << "| "<< map_.at(index).g <<"|";
        }
      }
    }
    std::cout << std::endl;

    std::cout << "rhs |";
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col; // Finding the appropriate cell in vectorized form
      //if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
      if (map_.at(index).cell.occupied) {
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      } else {
        //if(waveCells[index].getValue()<10 && waveCells[index].getValue()>=0)
        if (map_.at(index).rhs < 10 && map_.at(index).rhs >= 0) {
          std::cout << "|  "<< map_.at(index).rhs <<"|"; 
        } else if (map_.at(index).rhs == INT_MAX) {
          std::cout << "|"<< "inf" <<"|";
        } else {
          std::cout << "| "<< map_.at(index).rhs <<"|";
        }
      }
    }
    std::cout << "  " << row << std::endl;

    std::cout << "  h |";
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col; // Finding the appropriate cell in vectorized form
      //if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
      if (map_.at(index).cell.occupied) {
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      } else {
         //if(waveCells[index].getValue()<10 && waveCells[index].getValue()>=0)
        if (map_.at(index).cell.cost < 10 && map_.at(index).cell.cost >= 0) {
          std::cout << "|  "<< map_.at(index).cell.cost <<"|"; 
        } else if (map_.at(index).cell.cost == INT_MAX) {
          std::cout << "|"<< "inf" <<"|";
        } else {
          std::cout << "| "<< map_.at(index).cell.cost <<"|";
        }
      }
    }
    std::cout << std::endl;

    std::cout << "    |";
    for (int col = 0; col < GRID_WIDTH; ++col) {
      index = row * GRID_WIDTH + col; // Finding the appropriate cell in vectorized form
      //if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
      std::cout << " --- ";
      //if (map_.at(index).cell.occupied) {
      //  std::cout<<color_red<< "|  |" <<default_col<<"|";
      //} else {
      //   //if(waveCells[index].getValue()<10 && waveCells[index].getValue()>=0)
      //  if (map_.at(index).cell.cost < 10 && map_.at(index).cell.cost >= 0) {
      //    std::cout << "|  "<< map_.at(index).cell.cost <<"|"; 
      //  } else if (map_.at(index).cell.cost == INT_MAX) {
      //    std::cout << "|"<< "inf" <<"|";
      //  } else {
      //    std::cout << "| "<< map_.at(index).cell.cost <<"|";
      //  }
      //}
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
  //std::cout << "----------------------------------------------------------------" << std::endl;
}
