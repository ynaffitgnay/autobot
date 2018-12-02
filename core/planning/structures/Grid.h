#ifndef GRID_H
#define GRID_H

#include <vector>
#include <planning/PlanningConstants.h>
#include <planning/structures/GridCell.h>

//TODO: actually define grid. This is a placeholder for compilation
struct Grid {
  int width;
  int height;
  int cell_width;
  int cell_height;
  int columns;
  int rows;
  std::vector<GridCell> cells;
  Grid() : columns(GRID_WIDTH), rows(GRID_HEIGHT), cell_width(CELL_WIDTH), cell_height(CELL_HEIGHT),width(FIELD_WIDTH),height(FIELD_HEIGHT) {
    for (int r = 0; r < std::round((float)height/(float)cell_height); ++r) {
      for (int c = 0; c < std::round((float)width/(float)cell_width); ++c) {
        cells.push_back(GridCell(r,c));
      }
    }        
  }    
};
#endif
