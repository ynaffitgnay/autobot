#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#pragma once

#include <memory/MemoryCache.h>
#include <common/PathNode.h>
#include <planning/Logging.h>
#include <planning/structures/Grid.h>
#include <planning/PlanningConstants.h>
#include <common/PathNode.h>
#include <queue>
#include <vector>

class DStarLite {
  public:
    DStarLite(MemoryCache& cache, TextLogger*& tlogger);

    // TODO: move this out of here
    std::vector<std::vector<PathNode>> map_;

    
    std::priority_queue<PathNode> U_;
    int k_;  // key modifier
    const Grid* wf_;
  
    void init(const Grid& wavefront);
    void runDSL();
  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    int calcKey(PathNode successor);
    void updateVertex(PathNode u);
    void computeShortestPath();
    bool buildPathGrid();
};

#endif
