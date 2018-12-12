#include "CoverageDSL.h"
#include <iostream>
#include <list>
#include <memory/PlanningBlock.h>
#include <common/GridCell.h>

CoverageDSL::CoverageDSL(MemoryCache& cache, TextLogger*& tlogger) :
  DStarLite(tlogger), cache_(cache)
{
}

CoverageDSL::~CoverageDSL() {
  delete blankGrid;
}

void CoverageDSL::init(std::vector<GridCell>& wavefront, int startCoverageIdx) {
  DStarLite::init(wavefront, startCoverageIdx, -1, &(cache_.planning->path));
  buildBlankGrid();

  std::cout << "Map before doing dsl stuff: " << std::endl;

  printGrid();
  
  if (map_.size() <= 0) {
    std::cout << "EMPTY MAP IN RUNDSL?!\n" << std::endl;
    return;
  }

  std::vector<PathNode>::iterator mapIt;

  std::cout << "About to start computing shortest paths\n";

  // Initialize costs for each node to S_
  int calcNode = 0;
  //for (mapIt = map_.begin(); mapIt != map_.end(); mapIt++) {
  //  // Skip the goal node
  //  if (S_ == &(*mapIt)) {
  //    continue;
  //  }
  //      
  //  computeShortestPath(*mapIt);
  //  printGrid();
  //  std::cout << "Intermediate node expansions: " << nodeExpansions << std::endl;
  //  
  //  if (mapIt->g != mapIt->rhs) {
  //    //std::cout << "Cell at (" << mapIt->cell.r << ", " << mapIt->cell.c << ") was inconsistent" << std::endl;
  //    mapIt->g = mapIt->rhs;
  //  }    
  //}
  computeShortestPath(map_.at(46));

  std::cout << "Final map: \n" << std::endl;
  printGrid();
  
  std::cout << "About to generate path!\n";
  generateCoveragePath(0);
  initialized = true;
}

void CoverageDSL::runDSL() {
  std::vector<PathNode>::iterator mapIt;

  // Check if any edge costs have changed
  if (!cache_.planning->changedCost) {
    std::cout << "Unexpectedly in CoverageDSL::runDSL() with no detected changes" << std::endl;
  }

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
    std::cout << "Intermediate expanded nodes after replan: " << nodeExpansions << std::endl;
  }
  

  printGrid();

  // Mark all cells after the current idx unplanned (so beginning with pathIdx, inclusive)
  for (int i = cache_.planning->pathIdx; i < cache_.planning->nodesInPath; ++i) {
    map_.at(cache_.planning->path.at(i)).planned = false;
    map_.at(cache_.planning->path.at(i)).pathorder = -1;
  }
  cache_.planning->nodesInPath -= cache_.planning->nodesLeft;
    
  // Replan
  generateCoveragePath(cache_.planning->pathIdx);

  // Now that costs are consistent, reset the boolean in planning block
  cache_.planning->changedCost = false;
}

// This only works if we include the nodes discovered in "hop" in the path
// Determine the cost between two nodes on a path
int CoverageDSL::calcPathCost(int sIdx, int fIdx) {
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

//// Generate path from idx startIdx
void CoverageDSL::generateCoveragePath(int startIdx) {
  // Determine how many cells are to be in the full coverage plan
  int numPoses = 0;
  
  vector<PathNode>::iterator mapIt;
  for(mapIt = map_.begin(); mapIt != map_.end(); mapIt++)
  {
    // Cells to be planned do not include cells that could not be reached, obstructions, or cells planned in a previous plan
    if(mapIt->getValue() != INT_MAX && !mapIt->cell.occupied && !mapIt->cell.visited)
    {
      numPoses++;
    }
  }

  std::cout << "Poses to plan: " << numPoses << std::endl;

  auto& path = cache_.planning->path;
  int pathIdx = startIdx;
  int currCellIdx;
  
  if (startIdx == 0) {                  // Add the start pose as the first pose in the plan
    currCellIdx = S_->idx;              // index of the current cell being added to the plan
    S_->planned = true;
    S_->pathorder = pathIdx;
    path.at(pathIdx++) = currCellIdx;   
  } else {
    currCellIdx = cache_.planning->path.at(lastReplanIdx);
  }

  int newNumPlanned = 1;
  int numPlanned = newNumPlanned;
  while(newNumPlanned < numPoses)          // while we still have poses that need to be added to the plan
  {
    //std::cout << "PathIdx: " << pathIdx << std::endl;
    std::vector<PathNode*> unplanned;
    getUnplannedNeighbors(map_.at(currCellIdx), unplanned); // vector of valid unvisited neighbors
    if(unplanned.size() > 0)
    {
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
      //int lastIndex = currCellIdx;
      currCellIdx = unplanned.at(maxIndex)->idx;

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
        numPlanned = numPoses;
        //std::cout << "I guess we're stuck forever\n";
        break;
      }

      //std::cout << "Hop from " << lastIndex << " to " << currCellIdx << std::endl;

      std::vector<int>* hopPath = new std::vector<int>(GRID_SIZE);
      DStarLite* hopDSL = new DStarLite(tlogger_);
      hopDSL->init(*blankGrid, currCellIdx, lastIndex, hopPath);
      
      int hopsize = hopDSL->runDSL();
      if (hopsize == -1) {
        std::cout << "Uh oh... no path could be found to hop destination!" << std::endl;
        // TODO: can maybe have hop accept a list of indices that can't be hopped to!
      }

      delete(hopDSL);
 
      //std::cout << "hop path: ";
      //std::cout << "(" << getRowFromIdx(hopPath->at(0)) << ", " << getColFromIdx(hopPath->at(0)) << ") ";
      // Advance to the next spot in the path (since first one already in the path)
      for (int i = 1; i < hopsize; ++i) {
        //std::cout << "(" << getRowFromIdx(hopPath->at(i)) << ", " << getColFromIdx(hopPath->at(i)) << ") ";
        path.at(pathIdx++) = hopPath->at(i);
        ++numPlanned;
      }
      //std::cout << std::endl;
      
      delete(hopPath);
      
      map_.at(currCellIdx).pathorder = pathIdx - 1;
      map_.at(currCellIdx).planned = true;
      ++newNumPlanned;

      //std::cout << "pathIdx = " << pathIdx << " and numPlanned = " << numPlanned << std::endl;
    }
    
    //printPath();
  }
  printPath();
  
  printf("Path size: %d\n", numPlanned);
  printf("Node expansions in coverage: %d\n", nodeExpansions);

  //TODO: maybe sum the number of node expansions for vanilla DSL before deleting

  cache_.planning->nodesLeft = numPlanned;
  cache_.planning->nodesInPath += numPlanned;
}

void CoverageDSL::buildBlankGrid() {
  // TODO: add checks for robustness
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
    //std::cout << "checking (" << getRowFromIdx(s) << ", " << getColFromIdx(s) << ")" << std::endl;
    if (map_.at(s).planned == false) {
      index = s;
      found = true;
    }
    
    queue.pop_front();

    std::vector<PathNode*> neighbors;
    std::vector<PathNode*>::const_iterator neighborIt;
    //list<int>::const_iterator i;
    getNeighbors(map_.at(s), neighbors);

    for (neighborIt = neighbors.begin(); neighborIt != neighbors.end(); neighborIt++) {
      int thisIdx = (*neighborIt)->idx; 
      if (!checked[thisIdx]) {
        checked[thisIdx] = true;
        queue.push_back(thisIdx);
      } else {
        //std::cout << "Already checked  (" << getRowFromIdx(thisIdx) << ", " << getColFromIdx(thisIdx) << ")" << std::endl;
      }
    }
    
  }

  delete(checked);
      
  return index;
}
