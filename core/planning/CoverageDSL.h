#ifndef COVERAGE_D_STAR_LITE_H
#define COVERAGE_D_STAR_LITE_H

#pragma once

#include <memory/MemoryCache.h>
#include <vector>
#include <planning/DStarLite.h>

class CoverageDSL : public DStarLite {
  public:
    CoverageDSL(MemoryCache& cache, TextLogger*& tlogger);
    ~CoverageDSL();
    
    void init(std::vector<GridCell>& wavefront, int startCoverageIdx, bool = false);
    void runDSL();
    
  private:
    MemoryCache& cache_;
    
    std::vector<GridCell>* blankGrid;
    std::vector<int>* path_;
    bool AStar_;
    int calcPathCost(int sIdx, int fIdx);
    void generateCoveragePath(int startIdx);
    void buildBlankGrid();
    int hop(int index);
};

#endif
