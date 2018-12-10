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
    DStarLite(TextLogger*& tlogger);
    
    std::vector<PathNode> map_;
    std::vector<int>* path_;
    
    DSLPQueue U_;
    
    int k_;  // key modifier
    std::vector<GridCell>* cells_;
    PathNode* S_;  // Start/current location
    int goalIdx_;
    int endPlanIdx_;
  
    void init(std::vector<GridCell>& wavefront, int goal, int start, std::vector<int>* path);
    bool runDSL();

    int safeAdd(int q1, int q2);
  protected:
    TextLogger*& tlogger_;
    
    int lastReplanIdx;
    bool initialized;
    DSLKey calcKey(PathNode& successor);
    void updateVertex(PathNode& u);
    void computeShortestPath(PathNode& curr);
    void getNeighbors(PathNode& curr, std::vector<PathNode*>& neighbors);
    void getUnplannedNeighbors(PathNode& curr, vector<PathNode*>& neighbors);
    int getTransitionCost(PathNode& s, PathNode& p);
    bool generatePath(int startIdx);
    bool buildPathGrid();
    void printGrid();
    void printPath();
};

#endif
