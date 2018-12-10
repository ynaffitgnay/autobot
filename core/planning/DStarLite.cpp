#include "DStarLite.h"
#include <iostream>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/PlanningBlock.h>
#include <common/GridCell.h>

DStarLite::DStarLite(MemoryCache& cache, TextLogger*& tlogger, Point2D S)
  : cache_(cache), tlogger_(tlogger), startCoords_(S), cells_(nullptr), k_(0),
    lastReplanIdx(0), initialized(false) {
}
  
void DStarLite::init(std::vector<GridCell>& wavefront) {
  cells_ = &wavefront;
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

  U_ = DSLPQueue();

  S_->rhs = 0;

  // Calculate the key for the start node
  S_->key = calcKey(*S_);

  // Insert the start into the pqueue
  U_.push(S_);

  // Perform the initial cost calculations for each cell
  std::cout << "not initialized yet!\n";

  if (map_.size() <= 0) {
    std::cout << "EMPTY MAP IN RUNDSL?!\n" << std::endl;
    return;
  }

  std::vector<PathNode>::iterator mapIt;

  std::cout << "About to start computing shortest paths\n";

  // Initialize costs for each node to S_
  int calcNode = 0;
  for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
    if (S_ == &(*mapIt)) {
      //std::cout << "Skipping start point " << calcNode++ << std::endl;
      continue;
    }
    
    //if (mapIt->rhs != INT_MAX && mapIt->rhs == mapIt->g) {
    //  //std::cout << "Node cost for " << calcNode++ <<" (" << mapIt->cell.r << ", " << mapIt->cell.c << ") has already been calculated. It's " << mapIt->g << std::endl;
    //  continue;
    //}
    
    //std::cout << "Processing node (" << mapIt->cell.r << ", " << mapIt->cell.c << "), node " << calcNode++ << " / " << map_.size() << std::endl;
    
    computeShortestPath(*mapIt);
    // Print map after each iteration??
    //printGrid();
    // TODO: fix. check the edge
    if (mapIt->g != mapIt->rhs) {
      mapIt->g = mapIt->rhs;
    }
    
    //if (mapIt->g < mapIt->cell.cost) {
    //  std::cout << "HEURISTICS FOR CELL AT (" << mapIt->cell.r << ", " << mapIt->cell.c << ") INCONSISTENT\n\n\n\n";
    //  return;
    //}
    //std::cout << "\n\n";
  }

  std::cout << "Final map: \n" << std::endl;
  printGrid();
  
  std::cout << "About to generate path!\n";
  generateCoveragePath(0);
  initialized = true;
}

void DStarLite::runDSL() {
  //PathNode* S_last = nullptr;
  //PathNode* S_curr = nullptr;
  std::vector<PathNode>::iterator mapIt;

  // Check if any edge costs have changed
  if (!cache_.planning->changedCost) return;  

  // Get the new k offset
  // use pathIdx - 1 because we only want to replan up to visited node
  // (so pathIdx may actually be one further than the one we care about...)
  k_ = k_ + calcPathCost(lastReplanIdx, cache_.planning->pathIdx - 1);
  lastReplanIdx = cache_.planning->pathIdx - 1;
  S_ = &(map_.at(cache_.planning->path.at(lastReplanIdx)));  // Reset node planning from

  // Get a list of all edges with changed edge costs (basically any vertex that is
  // connected to a block w/ changed occupation)
  // For now, assume that the path with changed occupancy is the current destination
  // gridCell.
  std::vector<PathNode*> changedEdges;
  std::vector<PathNode*>::iterator changedIt;
  getNeighbors(map_.at(cache_.planning->path.at(cache_.planning->pathIdx)), changedEdges);

  for (changedIt = changedEdges.begin(); changedIt != changedEdges.end(); changedIt++) {
    updateVertex(**changedIt);
  }

  for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
    // Don't recalculate cost for visited cells
    if (mapIt->cell.visited) continue;

    computeShortestPath(*mapIt);
  }

  printGrid();

  // Mark all cells after the current idx unplanned (so beginning with pathIdx, inclusive)
  for (int i = cache_.planning->pathIdx; i < cache_.planning->nodesInPath; ++i) {
    //cache_.planning->grid.at(cache_.planning->path.at(i));
    map_.at(cache_.planning->path.at(i)).planned = false;
    map_.at(cache_.planning->path.at(i)).pathorder = -1;
  }
  cache_.planning->nodesInPath -= cache_.planning->nodesLeft;
    
  // Replan
  generateCoveragePath(cache_.planning->pathIdx);
 
  //std::cout << "in runDSL\n";

  // here we're going to need to update the DSL from a certain coordinate
  // TODO: update this function to use the robot's current location when replanning

  //TODO: make sure that no cells left on the path are occupied
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
    //std::cout << "Vertex is in the pq!" << std::endl;
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
      getNeighbors(*u, preds);

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
  return 7;
}

// This only works if we include the nodes discovered in "hop" in the path
// Determine the cost between two nodes on a path
int DStarLite::calcPathCost(int sIdx, int fIdx) {
  int traversalCost = 0;

  // Go through each node between these two path indices and sum the transition cost between them
  // Also check h(sIdx) - h(fIdx) to see what that is...
  int i;
  for (i = sIdx; i < fIdx; ++i) {
    // TODO: make sure that this is within bounds...
    traversalCost += getTransitionCost(map_.at(cache_.planning->path.at(i)), map_.at(cache_.planning->path.at(i + 1)));
  }

  std::cout << "Got cost up to " << i << " to " << i + 1 << std::endl;
  
  return traversalCost;
}

// Generate path from idx startIdx
void DStarLite::generateCoveragePath(int startIdx) {
  // Determine how many cells are to be in the full coverage plan
  //int numPoses = 0;     // number of poses we need to add to the plan
  // TODO: make this the total number of unoccupied cells - the ones that have been visited...
  int numPoses = 0;
  vector<PathNode>::iterator mapIt;
  for(mapIt = map_.begin(); mapIt != map_.end(); mapIt++)
  {
    // Cells to be planned do not include cells that the wavefront propogation could not reach, obstructions, or cells planned in a previous plan
    if(mapIt->getValue() != INT_MAX && !mapIt->cell.occupied && !mapIt->cell.visited)
    {
      numPoses++;
    }
  }

  std::cout << "Poses to plan: " << numPoses << std::endl;

  auto& path = cache_.planning->path;
  int pathIdx = startIdx;
  int currCellIdx;
  
  // Plan order of poses
  if (startIdx == 0) {
    currCellIdx = S_->idx;                    // index of the current cell being added to the plan
    S_->planned = true;
    S_->pathorder = pathIdx;
    path.at(pathIdx++) = currCellIdx;   // Add the start pose as the first pose in the plan
  } else {
    currCellIdx = cache_.planning->path.at(lastReplanIdx);
  }
  
  //waveCells[end_index_].visit();               // visit the end cell so it will not be planned (it will always be added at the end)
  int numPlanned = 1;                        // start and end cell
  while(numPlanned < numPoses)               // while we still have poses that need to be added to the plan
  {
    //bool stuck = false;
    //while(!stuck && numPlanned <= numPoses)  // if we get stuck, we exit the loop and use the hop function to find the closest unplanned cell
    //{
    std::vector<PathNode*> unplanned;
    getUnplannedNeighbors(map_.at(currCellIdx), unplanned); // vector of valid unvisited neighbors
    if(unplanned.size() > 0) // We can only move to a neighbor if it is valid and unplanned
    {
      int maxValue = unplanned[0]->getValue(); // value of the neighbor whose wavefront number is the highest
      int maxIndex = 0;                       // index of the neighbor whose wavefront number is the highest
      if(unplanned.size() > 1) // If there is more than one unplanned neighbor, find the one with the highest wavefront value
      {
        for(int i = 1; i < unplanned.size(); i++)
        {
          if(unplanned[i]->getValue() > maxValue)
          {
            maxValue = unplanned[i]->getValue();
            maxIndex = i;
          }
        }
      }
      int lastIndex = currCellIdx;
      currCellIdx = unplanned.at(maxIndex)->idx;

      map_.at(currCellIdx).pathorder = pathIdx;
      map_.at(currCellIdx).planned = true;
      path.at(pathIdx++) = currCellIdx;
      numPlanned++;
    }
    else // if all valid neighbors have been visited, we are stuck
    {
    //  //printf("Stuck: no neighbors are unplanned.\n");
    //  stuck = true;
    //}
    //
    //if (stuck)
    //{
      // When we get stuck, find the closest unplanned cell and go there
      int lastIndex = currCellIdx;
      currCellIdx = hop(currCellIdx); // hop to the closed unplanned cell
      if(currCellIdx == lastIndex) {// if hop was unsuccessful, we should must not have any more cells to plan
        std::cout << "Hop unsuccessful. Original numPlanned: " << numPlanned <<
          " and numPoses: " << numPoses << std::endl;
        numPlanned = numPoses;
        //std::cout << "I guess we're stuck forever\n";
        break;
      }
      map_.at(currCellIdx).pathorder = pathIdx;
      map_.at(currCellIdx).planned = true;
      path.at(pathIdx++) = currCellIdx;
      numPlanned++;
    }
      //}
  }
  printPath();
  
  printf("Path size: %d\n", numPlanned);

  cache_.planning->nodesLeft = numPlanned;
  cache_.planning->nodesInPath += numPlanned;
}

bool DStarLite::buildPathGrid() {
  // TODO: add checks for robustness
  if (cells_->size() <= 0) {
    std::cout << "Wavefront grid is empty" << std::endl;
    return false;
  }

  // Create empty gridcells for planning paths between stuck points
  for (int r = 0; r < GRID_HEIGHT; ++r) {
    for (int c = 0; c < GRID_WIDTH; ++c) {
      blankGrid.push_back(GridCell(r,c));
    }
  }

  std::vector<GridCell>::iterator gridIt;
  int gridIdx = 0;
  
  for (gridIt = cells_->begin(); gridIt != cells_->end(); gridIt++) {
    if (gridIt->occupied) {
      blankGrid.at(gridIdx).occupied = true;
    }
    PathNode cell = PathNode(*(gridIt));
    map_.push_back(cell);
    ++gridIdx;
  }

  if (map_.size() != GRID_SIZE) {
    std::cout << "map_.size(): " << map_.size() << " GRID_SIZE: " << GRID_SIZE << std::endl;
    std::cout << "GRID_SIZE ALL WRONG WHEN BUILDING PATH_GRID???" << std::endl;
  }
  
  return true;
}

int DStarLite::hop(int index)
{
  // Find the closest unvisited cell
  bool found = false;        // true if we have found the closest unvisited cell
  std::vector<int> checked;  // vector of indices of cells that we have already checked to be unvisited
  checked.clear();
  checked.push_back(index);  // put the current cell on the list since it should hop back to itself
  std::vector<PathNode*> neighbors; // vector of neighbors propogating outward from original cell
  neighbors.clear();
  neighbors.push_back(&map_.at(index)); // we start propogation from the cell we got stuck at
  while(!found && neighbors.size() != 0) // keep going until we find a valid hop, or we have checked all valid cells and therefore neighbors is empty
  {
    int numOld = neighbors.size(); // number of neighbors in the previous iteration
    int i = 0; // iteration of old neighbors
    while(i < numOld && !found) // iterate all old neighbors unless we find a valid cell to hop to
    {
      std::vector<PathNode*> newNeighbors;
      getNeighbors(*neighbors.at(i), newNeighbors); // neighbors of our old neighbors (this is how we propogate outward)
      int j = 0; // iteration of new neighbors
      while(j < newNeighbors.size() && !found) // check all new neighbors unless one of them is found to be a valid hop
      {
        if(std::find(checked.begin(), checked.end(), newNeighbors.at(j)->idx) == checked.end()) // if the new neighbor has not already been checked
        {
          int r, c;
          //getCoordinate(getIndex(newNeighbors[j]->getPosition()), r, c);
          neighbors.push_back(newNeighbors.at(j)); // move the new neighbor to the old neighbors list so that in the next iteration we check its neighbors
          //checked.push_back(getIndex(newNeighbors[j]->getPosition())); // add this neighbor to the already checked list
          checked.push_back(newNeighbors.at(j)->idx);
          if(!newNeighbors.at(j)->planned)//->isVisited())  // if it has not been visited, we found a valid hop
          {
            index = newNeighbors.at(j)->idx;//getIndex(newNeighbors[j]->getPosition());
            found = true;
          }
        }
        j++;
      }
      i++;
    }
    
    if(numOld == 1) { // delete the neighbors who ran through the algorithm in the last iteration so we don't keep rechecking neighbors
      neighbors.erase(neighbors.begin());
    } else {
      neighbors.erase(neighbors.begin(), neighbors.begin() + numOld);
    }
  }
  
  if(!found && neighbors.size() == 0) // the algorithm failed (this means we have already planned all poses and didn't need to hop in the first place)
  {
    //printf("We ran out of neighbors. Hop was called when all poses had already been planned\n");
  }
    
  return index;
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
    std::cout << "    |";
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
