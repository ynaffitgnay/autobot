#ifndef PLANNING_BLOCK_H
#define PLANNING_BLOCK_H
#pragma once

#include <common/PathNode.h>
#include <memory/MemoryBlock.h>
#include <math/Geometry.h>
#include <vector>
#include <planning/PlanningConstants.h>
#include <schema/gen/PlanningBlock_generated.h>

DECLARE_INTERNAL_SCHEMA(struct PlanningBlock : public MemoryBlock {
  public:
    SCHEMA_METHODS(PlanningBlock);
    PlanningBlock();
    SCHEMA_FIELD(Point2D start_point);

    mutable SCHEMA_FIELD(std::array<PathNode, GRID_SIZE> grid_data);
    std::vector<PathNode> grid;

  SCHEMA_PRE_SERIALIZATION({
      std::copy(
        __source_object__.grid.data(), 
        __source_object__.grid.data() + __source_object__.grid.size(), 
        __source_object__.grid_data.data()
      );
  });
  SCHEMA_POST_DESERIALIZATION({
      std::copy(
        __target_object__.grid_data.data(), 
        __target_object__.grid_data.data() + __target_object__.grid.size(),
        __target_object__.grid.data()
      );
  });

});

#endif
