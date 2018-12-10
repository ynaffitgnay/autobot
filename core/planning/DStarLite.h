#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#pragma once

#include <memory/MemoryCache.h>
#include <math/Geometry.h>
#include <common/DSLKey.h>
#include <planning/Logging.h>
#include <planning/structures/PNCmp.h>
#include <planning/structures/Grid.h>
#include <planning/structures/PathNode.h>
#include <planning/structures/DSLPQueue.h>
#include <planning/PlanningConstants.h>
#include <queue>
#include <vector>

class DStarLite {
  public:
    DStarLite(MemoryCache& cache, TextLogger*& tlogger, Point2D startloc);
    
    std::vector<PathNode> map_;
    
    DSLPQueue U_;
    
    int k_;  // key modifier
    std::vector<GridCell>* cells_;
    Point2D startCoords_;
    PathNode* S_;  // Start/current location
    int startIdx_;
    int endIdx_;
    //PathNode* S_prev;  // Previous location
  
    void init(std::vector<GridCell>& wavefront, int start, int end);
    void runDSL();

    int safeAdd(int q1, int q2);
  protected:
    MemoryCache& cache_;
    TextLogger*& tlogger_;

    //std::vector<GridCell> blankGrid;
    
    int lastReplanIdx;
    bool initialized;
    DSLKey calcKey(PathNode& successor);
    void updateVertex(PathNode& u);
    void computeShortestPath(PathNode& curr);
    void getNeighbors(PathNode& curr, std::vector<PathNode*>& neighbors);
    void getUnplannedNeighbors(PathNode& curr, vector<PathNode*>& neighbors);
    int getTransitionCost(PathNode& s, PathNode& p);
    int calcPathCost(int sIdx, int fIdx);
    void generateCoveragePath(int startIdx);
    bool buildPathGrid();
    int hop(int index);
    void printGrid();
    void printPath();
};

#endif
