#ifndef PLANNING_MODULE_H
#define PLANNING_MODULE_H

#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <memory/PlanningBlock.h>
#include <planning/CoverageDSL.h>
#include <planning/WavefrontPropagation.h>
#include <planning/GridGenerator.h>
#include <planning/structures/Grid.h>
#include <common/GridCell.h>

class VisionCore;

class PlanningModule: public Module {
  public:
    PlanningModule();
    ~PlanningModule();

    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void processFrame();

    inline std::vector<GridCell>& grid() {
      return cache_.planning->grid;
    }

  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    Grid* initial_cost_map_;
    Pose2D wfStartPose;
    std::unique_ptr<GridGenerator> GG_;
    std::unique_ptr<WavefrontPropagation> WP_;
    std::unique_ptr<CoverageDSL> DSL_;

    Point2D startLoc_;
    int prevLoc_r;
    int prevLoc_c;

    void updateCell();
};

#endif
