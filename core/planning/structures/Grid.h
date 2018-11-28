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
    for (int y = 0; y < height; ++y) {
      std::vector<GridCell> row;
      for (int x = 0; x < width; ++x) {
        row.push_back(GridCell(x - GRID_WIDTH / 2, y - GRID_HEIGHT / 2));
      }
      cells.push_back(row);
    }
        
  }
    
};
#endif
