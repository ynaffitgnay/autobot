#ifndef GRID_CELL_H
#define GRID_CELL_H

#pragma once

// TODO: fix this definition
struct GridCell {
  int x;
  int y;
  int cost;

  GridCell(int x, int y) : x(x), y(y), cost(0) { };
};


#endif
