#ifndef GRID_CELL_H
#define GRID_CELL_H

#pragma once

// TODO: fix this definition
struct GridCell {
  int r;
  int c;
  int cost;
  bool occupied;

  GridCell(int row, int col) : r(row), c(col), cost(0) { };
};

#endif
