#ifndef GRID_CELL_H
#define GRID_CELL_H

#pragma once

// TODO: fix this definition
struct GridCell {
  int r;
  int c;
  Pose2D center;
  int cost;
  bool occupied;

  GridCell(int row, int col) : r(row), c(col), cost(0), occupied(false) {
  	int centerX = CELL_WIDTH*col+CELL_WIDTH/2;
  	int centerY = CELL_HEIGHT*row+CELL_HEIGHT/2;
  	center.translation.x = centerX;
  	center.translation.y = centerY;
  	center.rotation = 0.0;

  };
};

#endif
