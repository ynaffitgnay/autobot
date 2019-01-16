#include "CoverageDSL.h"
#include <iostream>
#include <list>
#include <memory/PlanningBlock.h>
#include <common/GridCell.h>

CoverageDSL::CoverageDSL(MemoryCache& cache, TextLogger*& tlogger) :
  DStarLite(tlogger), cache_(cache), AStar_(false)
{
}

CoverageDSL::~CoverageDSL() {
  delete blankGrid;
}

void CoverageDSL::init(std::vector<GridCell>& wavefront, int startCoverageIdx, bool AStar) {
  AStar_ = AStar;
  DStarLite::init(wavefront, startCoverageIdx, -1, &(cache_.planning->path));
  buildBlankGrid();
  // 0 paths have been planned!
  
  if (!AStar_) cache_.planning->pathsPlanned = 0;
  if (map_.size() <= 0) {
    std::cout << "EMPTY MAP IN RUNDSL?!\n" << std::endl;
    return;
  }

  std::vector<PathNode>::iterator mapIt;

  std::cout << "About to start computing shortest paths\n";

  // Initialize costs for each node to S_
  int calcNode = 0;
  for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
    // Skip the goal node
    if (S_ == &(*mapIt)) {
      continue;
    }
        
    computeShortestPath(*mapIt);
    //printGrid();
    //std::cout << "Intermediate node expansions: " << nodeExpansions << std::endl;
    
    if (mapIt->g != mapIt->rhs) {
      mapIt->g = mapIt->rhs;
    }    
  }

  std::cout << "Final map: \n" << std::endl;
  printGrid();
  
  std::cout << "About to generate path!\n";
  if (AStar_) {
    // Need to mark the ones up to this point as planned
    for (int i = 0; i < cache_.planning->pathIdx; ++i) {
      map_.at(cache_.planning->path.at(i)).planned = true;
      map_.at(cache_.planning->path.at(i)).pathorder = i;
  }
    generateCoveragePath(cache_.planning->pathIdx);
  } else {
    generateCoveragePath(0);
  }
}

void CoverageDSL::runDSL() {
  std::vector<PathNode>::iterator mapIt;

  // Check if any edge costs have changed
  if (!cache_.planning->changedCost) {
    std::cout << "Unexpectedly in CoverageDSL::runDSL() with no detected changes" << std::endl;
  }

  // Mark this cell as occupied
  PathNode& changedNode = map_.at(cache_.planning->path.at(cache_.planning->pathIdx));
  changedNode.cell.occupied = true;
  changedNode.changed = true;
  blankGrid->at(cache_.planning->path.at(cache_.planning->pathIdx)).occupied = true;

  // Get the new k offset
  // use pathIdx - 1 because we only want to replan up to visited node
  k_ = k_ + calcPathCost(lastReplanIdx, cache_.planning->pathIdx - 1);
  lastReplanIdx = cache_.planning->pathIdx - 1;
  S_ = &(map_.at(cache_.planning->path.at(lastReplanIdx)));  // Reset node planning from

  // Get a list of all edges with changed edge costs (basically any vertex that is
  // connected to a block w/ changed occupation)
  // Assume that the next cell on the path is the one that changed occupation
  std::vector<PathNode*> changedEdges;
  std::vector<PathNode*>::iterator changedIt;
  getNeighbors(changedNode, changedEdges);
  ++nodeExpansions;

  for (changedIt = changedEdges.begin(); changedIt != changedEdges.end(); changedIt++) {
    int c_old = getPrevTransitionCost(changedNode, **changedIt);
    int c_new = getTransitionCost(changedNode, **changedIt);
    if (c_old > c_new) {
      (*changedIt)->rhs = std::min((*changedIt)->rhs, safeAdd(c_new, changedNode.g));
    } else if ((*changedIt)->rhs == safeAdd(c_old, changedNode.g)) {
      if (*changedIt != S_) {
        int minrhs = INT_MAX;
        std::vector<PathNode*> succs;
        std::vector<PathNode*>::iterator succIt;

        getNeighbors(**changedIt, succs);
        ++nodeExpansions;
        for (succIt = succs.begin(); succIt != succs.end(); succIt++) {
          int t2 = safeAdd(getTransitionCost(**changedIt, **succIt), (*succIt)->g); 
          if (t2 < minrhs) {
            minrhs = t2;
          }
        }
        (*changedIt)->rhs = minrhs;
      }
    }
    
    updateVertex(**changedIt);
  }

  // Now mark this edge as no longer changed
  map_.at(cache_.planning->path.at(cache_.planning->pathIdx)).changed = false;

  for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
    // Don't recalculate cost for visited cells
    if (mapIt->cell.visited) continue;

    computeShortestPath(*mapIt);
    std::cout << "Intermediate expanded nodes after replan: " << nodeExpansions << std::endl;
  }
  

  printGrid();

  // Mark all cells after the current idx unplanned if they haven't been visited
  // (so beginning with pathIdx, inclusive)
  for (int i = cache_.planning->pathIdx; i < cache_.planning->nodesInPath; ++i) {
    if (!map_.at(cache_.planning->path.at(i)).cell.visited) {
      map_.at(cache_.planning->path.at(i)).planned = false;
      map_.at(cache_.planning->path.at(i)).pathorder = -1;
    }
  }
  // Note that this only gets run for DStarLite, not for AStar
  cache_.planning->nodesInPath = cache_.planning->pathIdx - 1;
  
    
  // Replan
  generateCoveragePath(cache_.planning->pathIdx);

  // Now that costs are consistent, reset the boolean in planning block
  cache_.planning->changedCost = false;

  // Set the number of node expansions
  if (AStar_) {
    cache_.planning->nodeExpansions += nodeExpansions;
  } else {
    cache_.planning->nodeExpansions = nodeExpansions;
  }
}

// This only works if we include the nodes discovered in "hop" in the path
// Determine the cost between two nodes on a path
int CoverageDSL::calcPathCost(int sIdx, int fIdx) {
  int traversalCost = 0;

  int i;
  for (i = sIdx; i < fIdx; ++i) {
    traversalCost += getTransitionCost(map_.at(cache_.planning->path.at(i)), map_.at(cache_.planning->path.at(i + 1)));
  }

  int h_diff = map_.at(cache_.planning->path.at(fIdx)).cell.cost - map_.at(cache_.planning->path.at(sIdx)).cell.cost;
  
  return h_diff;
}

// Generate path from idx startIdx
void CoverageDSL::generateCoveragePath(int startIdx) {
  // Determine how many cells are to be in the full coverage plan
  int numPoses = 0;

  std::set<int> uset;
  std::set<int>::iterator uIt;
  
  vector<PathNode>::iterator mapIt;
  for(mapIt = map_.begin(); mapIt != map_.end(); mapIt++)
  {
    // Cells to be planned do not include cells that could not be reached, obstructions, or cells planned in a previous plan
    if(mapIt->getValue() != INT_MAX && !mapIt->cell.occupied && !mapIt->cell.visited)
    {
      numPoses++;
      uset.insert(mapIt->idx);
    }
    
  }

  std::cout << "Poses to plan: " << numPoses << std::endl;

  auto& path = cache_.planning->path;
  int pathIdx = startIdx;
  int newNumPlanned;
  int currCellIdx;
  
  if (startIdx == 0) {                  // Add the start pose as the first pose in the plan
    currCellIdx = S_->idx;              // index of the current cell being added to the plan
    S_->planned = true;
    S_->pathorder = pathIdx;
    path.at(pathIdx++) = currCellIdx;
    newNumPlanned = 1;
  } else {
    currCellIdx = (AStar_) ? cache_.planning->path.at(cache_.planning->pathIdx - 1) : cache_.planning->path.at(lastReplanIdx);
    
    newNumPlanned = 0;
  }

  int numPlanned = newNumPlanned;
  while(newNumPlanned < numPoses)          // while we still have poses that need to be added to the plan
  {
    //std::cout << "PathIdx: " << pathIdx << std::endl;
    std::vector<PathNode*> unplanned;
    getUnplannedNeighbors(map_.at(currCellIdx), unplanned); // vector of valid unvisited neighbors
    if(unplanned.size() > 0)
    {
      if (!AStar_) { // AStar_ uses gradient ascent
        int minValue = unplanned[0]->getValue();
        int minIndex = 0;                       
        if(unplanned.size() > 1) // If there is more than one unplanned neighbor, find the one with the highest cost
        {
          for(int i = 1; i < unplanned.size(); i++)
          {
            if(unplanned[i]->getValue() < minValue)
            {
              minValue = unplanned[i]->getValue();
              minIndex = i;
            }
          }
        }
        currCellIdx = unplanned.at(minIndex)->idx;
      } else {
        int maxValue = unplanned[0]->getValue();
        int maxIndex = 0;                       
        if(unplanned.size() > 1) // If there is more than one unplanned neighbor, find the one with the highest cost
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
        currCellIdx = unplanned.at(maxIndex)->idx;
      }
        

      if (map_.at(currCellIdx).cell.occupied) {
        std::cout << "An unexpected occupation" << std::endl;
      }

      uIt = uset.find(currCellIdx);
      uset.erase(uIt);

      map_.at(currCellIdx).pathorder = pathIdx;
      
      map_.at(currCellIdx).planned = true;
      path.at(pathIdx++) = currCellIdx;
      ++newNumPlanned;
      ++numPlanned;
    }
    else // Check for another valid neighbor
    {
      // When we get stuck, find the closest unplanned cell and go there
      int lastIndex = currCellIdx;
      currCellIdx = hop(currCellIdx); // hop to the closed unplanned cell
      if(currCellIdx == lastIndex) {  // if hop was unsuccessful, we should must not have any more cells to plan
        std::cout << "Hop unsuccessful. Original numPlanned: " << numPlanned <<
          " and numPoses: " << numPoses << std::endl;
        //numPlanned = numPoses;
        numPoses = numPlanned;
        break;
      }

      std::vector<int>* hopPath = new std::vector<int>(GRID_SIZE);
      DStarLite* hopDSL = new DStarLite(tlogger_);
      hopDSL->init(*blankGrid, currCellIdx, lastIndex, hopPath);
      
      int hopsize = hopDSL->runDSL();
      if (hopsize == -1) {
        std::cout << "Uh oh... no path could be found to hop destination!" << std::endl;
      }

      std::cout << "Adding " << hopDSL->nodeExpansions << " nodeExpansions to curr" << std::endl;
      nodeExpansions += hopDSL->nodeExpansions;

      delete(hopDSL);
 
      // Advance to the next spot in the path (since first one already in the path)
      for (int i = 1; i < hopsize; ++i) {
        if (map_.at(hopPath->at(i)).cell.occupied) {
          std::cout << "blankGrid not getting updated properly" << std::endl;
        }
        if (!map_.at(hopPath->at(i)).planned) {
          uIt = uset.find(currCellIdx);

          uset.erase(uIt);
          map_.at(hopPath->at(i)).planned = true;
          map_.at(hopPath->at(i)).pathorder = pathIdx;
          ++newNumPlanned;
        }
        path.at(pathIdx++) = hopPath->at(i);
        ++numPlanned;
      }
      
      delete(hopPath);
      
      map_.at(currCellIdx).pathorder = pathIdx - 1;
      map_.at(currCellIdx).planned = true;
    }
  }
  
  printPath();
  
  printf("Path size: %d\n", numPlanned);
  printf("Node expansions in coverage: %d\n", nodeExpansions);

  cache_.planning->nodesLeft = numPlanned;
  if (AStar_) {
    cache_.planning->nodesInPath = numPlanned + cache_.planning->pathIdx;
    cache_.planning->nodeExpansions += nodeExpansions;
  } else {
    cache_.planning->nodesInPath += numPlanned;
    cache_.planning->nodeExpansions = nodeExpansions;
  }
  cache_.planning->pathsPlanned += 1;
}

void CoverageDSL::buildBlankGrid() {
  if (cells_->size() <= 0) {
    std::cout << "Wavefront grid is empty" << std::endl;
    return;
  }

  blankGrid = new std::vector<GridCell>();

  // Create empty gridcells for planning paths between stuck points
  for (int r = 0; r < GRID_HEIGHT; ++r) {
    for (int c = 0; c < GRID_WIDTH; ++c) {
      blankGrid->push_back(GridCell(r,c));
    }
  }

  std::vector<GridCell>::iterator gridIt;
  int gridIdx = 0;
  
  for (gridIt = cells_->begin(); gridIt != cells_->end(); gridIt++) {
    if (gridIt->occupied) {
      blankGrid->at(gridIdx).occupied = true;
    }
    ++gridIdx;
  }
}

int CoverageDSL::hop(int index)
{
  // Find the closest unvisited cell
  bool found = false;        // true if we have found the closest unvisited cell
  bool* checked = new bool[GRID_SIZE];

  std::fill_n(checked, GRID_SIZE, false);

  std::list<int> queue;
  checked[index] = true;

  queue.push_back(index);
  
  while (!queue.empty() && !found) {
    int s = queue.front();
    if (map_.at(s).planned == false) {
      index = s;
      found = true;
    }
    
    queue.pop_front();

    std::vector<PathNode*> neighbors;
    std::vector<PathNode*>::const_iterator neighborIt;
    getNeighbors(map_.at(s), neighbors);

    for (neighborIt = neighbors.begin(); neighborIt != neighbors.end(); neighborIt++) {
      int thisIdx = (*neighborIt)->idx; 
      if (!checked[thisIdx]) {
        checked[thisIdx] = true;
        queue.push_back(thisIdx);
      }
    }
    
  }

  delete(checked);
      
  return index;
}
