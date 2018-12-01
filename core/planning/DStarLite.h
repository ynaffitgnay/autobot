#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#pragma once

#include <memory/MemoryCache.h>
#include <math/Geometry.h>
#include <common/DSLKey.h>
#include <common/PathNode.h>
#include <planning/Logging.h>
#include <planning/structures/PNCmp.h>
#include <planning/structures/Grid.h>
#include <planning/structures/DSLPQueue.h>
#include <planning/PlanningConstants.h>
#include <queue>
#include <vector>

class DStarLite {
  public:
    DStarLite(MemoryCache& cache, TextLogger*& tlogger, Point2D startloc);

    // TODO: move this out of here
    std::vector<std::vector<PathNode>> map_;
    
    //priority_queue<PathNode*, vector<PathNode*>, PNCmp> U_;
    DSLPQueue U_;
    int k_;  // key modifier
    const Grid* wf_;
    Point2D startCoords_;
    PathNode* S_;  // Start/current location
    //int S_r_;  // Start node row
    //int S_c_;  // Start node column
    //int S_x; // Start x-coord
    //int S_y; // Start y-coord
  
    void init(const Grid& wavefront);
    void runDSL();
    //void runCCDSL();
    //void runDSL(PathNode& dest);
    //static int safeAdd(int q1, int q2);
    int safeAdd(int q1, int q2);
  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    DSLKey calcKey(PathNode& successor);
    void updateVertex(PathNode& u);
    void computeShortestPath(PathNode& curr);
    void getPreds(PathNode& successor, std::vector<PathNode*>& preds);
    void getSuccs(PathNode& predecessor, std::vector<PathNode*>& succs);
    int getTransitionCost(PathNode& s, PathNode& p);
    bool buildPathGrid();
};

#endif
