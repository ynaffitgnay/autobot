#ifndef GRID_H
#define GRID_H

#include <vector>
#include <planning/PlanningConstants.h>
#include <planning/structures/GridCell.h>

//TODO: actually define grid. This is a placeholder for compilation
struct Grid {
  int width;
  int height;
  std::vector<GridCell> cells;
  Grid() : width(GRID_WIDTH), height(GRID_HEIGHT), cell_width(CELL_WIDTH), cell_height(CELL_HEIGHT) {
    for (int r = 0; r < std::round(height/cell_height); ++r) {
      for (int c = 0; c < std::round(width/cell_width); ++c) {
        cells.push_back(GridCell(r,c));
      }
    }        
  }    
};
#endif
