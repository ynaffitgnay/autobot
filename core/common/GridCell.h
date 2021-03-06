#ifndef GRID_CELL_H
#define GRID_CELL_H

#pragma once

#include <math/Pose2D.h>
#include <common/Serialization.h>
#include <planning/PlanningConstants.h>
#include <schema/gen/GridCell_generated.h>

DECLARE_INTERNAL_SCHEMA(struct GridCell {
  SCHEMA_METHODS(GridCell);
  SCHEMA_FIELD(int r);
  SCHEMA_FIELD(int c);
  SCHEMA_FIELD(Pose2D center);
  SCHEMA_FIELD(int cost);
  SCHEMA_FIELD(bool occupied);
  SCHEMA_FIELD(bool visited);
  SCHEMA_FIELD(int order_index);

  GridCell() : r(-1), c(-1), cost(0), occupied(false), order_index(-1),
    visited(false) {};

  GridCell(int row, int col) : r(row), c(col), cost(0), occupied(false),
    visited(false), order_index(-1)
  {
  	int centerX = CELL_WIDTH * col + CELL_WIDTH / 2;  
  	int centerY = CELL_HEIGHT * row + CELL_HEIGHT / 2;
  	center.translation.x = centerX;
  	center.translation.y = centerY;
  	center.rotation = 0.0;
  };

  GridCell(const GridCell& other) : r(other.r), c(other.c), center(other.center),
    cost(other.cost), occupied(other.occupied), visited(other.visited),
    order_index(other.order_index) { };
});
#endif
