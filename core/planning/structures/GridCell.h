#ifndef GRID_CELL_H
#define GRID_CELL_H
#include <math/Pose2D.h>
#include <planning/PlanningConstants.h>

#pragma once

// TODO: fix this definition
struct GridCell {
  int r;
  int c;
  Pose2D center;
  int cost;
  bool occupied;

  GridCell() : r(-1), c(-1), cost(0), occupied(false) {};

  GridCell(int row, int col) : r(row), c(col), cost(0), occupied(false) {
  	int centerX = CELL_WIDTH*col+CELL_WIDTH/2;
  	int centerY = CELL_HEIGHT*row+CELL_HEIGHT/2;
  	center.translation.x = centerX;
  	center.translation.y = centerY;
  	center.rotation = 0.0;
  };
};

#endif
