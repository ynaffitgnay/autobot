#ifndef GRID_H
#define GRID_H

#include <vector>
#include <planning/PlanningConstants.h>
#include <planning/structures/GridCell.h>

//TODO: actually define grid. This is a placeholder for compilation
struct Grid {
  int width;
  int height;
  std::vector<vector<GridCell>> cells;
  // TODO: reset so that the center of the field = 0,0
  Grid() : width(GRID_WIDTH), height(GRID_HEIGHT) {
    for (int r = 0; r < height; ++r) {
      std::vector<GridCell> row;
      for (int c = 0; c < width; ++c) {
        row.push_back(GridCell(r, c));
      }
      cells.push_back(row);
    }
  }
};
#endif
