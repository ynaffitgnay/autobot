#ifndef PLANNING_BLOCK_H
#define PLANNING_BLOCK_H
#pragma once

//#include <common/PathNode.h>
#include <memory/MemoryBlock.h>
#include <math/Geometry.h>
#include <math/Pose2D.h>
#include <vector>
#include <planning/PlanningConstants.h>
#include <planning/structures/GridCell.h>
#include <schema/gen/PlanningBlock_generated.h>

DECLARE_INTERNAL_SCHEMA(struct PlanningBlock : public MemoryBlock {
  public:
    SCHEMA_METHODS(PlanningBlock);
    PlanningBlock();
    SCHEMA_FIELD(Point2D startPoint);  // Set desired starting point
    SCHEMA_FIELD(bool coverageStarted);
    SCHEMA_FIELD(bool changedCost);
    SCHEMA_FIELD(int pathIdx);

    mutable SCHEMA_FIELD(std::array<GridCell, GRID_SIZE> grid_data);
    std::vector<GridCell> grid;

    mutable SCHEMA_FIELD(std::array<GridCell, PATH_SIZE> path_data);
    std::vector<GridCell> path;

  SCHEMA_PRE_SERIALIZATION({
      std::copy(
        __source_object__.grid.data(), 
        __source_object__.grid.data() + __source_object__.grid.size(), 
        __source_object__.grid_data.data()
      );
      std::copy(
        __source_object__.path.data(), 
        __source_object__.path.data() + __source_object__.path.size(), 
        __source_object__.path_data.data()
      );
  });
  SCHEMA_POST_DESERIALIZATION({
      std::copy(
        __target_object__.grid_data.data(), 
        __target_object__.grid_data.data() + __target_object__.grid.size(),
        __target_object__.grid.data()
      );
      std::copy(
        __target_object__.path_data.data(), 
        __target_object__.path_data.data() + __target_object__.path.size(),
        __target_object__.path.data()
      );
  });

  Pose2D getDestPose();
});

#endif
