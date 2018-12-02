#ifndef PLANNING_MODULE_H
#define PLANNING_MODULE_H

#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <planning/DStarLite.h>
#include <planning/WavefrontPropagation.h>
#include <planning/GridGenerator.h>
#include <planning/structures/Grid.h>

class VisionCore;

class PlanningModule: public Module {
  public:
    PlanningModule();

    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void processFrame();

  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    
    std::unique_ptr<GridGenerator> GG_;
    std::unique_ptr<WavefrontPropagation> WP_;
    std::unique_ptr<DStarLite> DSL_;
    Grid initial_cost_map_;

    void visitNewCell();
};

#endif
