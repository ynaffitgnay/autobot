#include <planning/GridGenerator.h>

GridGenerator::GridGenerator() {
}

bool GridGenerator::generateGrid(Grid& grid) {
  for (auto& cell : grid.cells) {
		cell.occupied = checkObstruction(cell);
		// printf("Processing cell at [%d,%d]. Occupied: %d\n",cell.r,cell.c, cell.occupied);
	}
	return true;
}

bool GridGenerator::checkObstruction(GridCell& gc) {
  for (auto& obs : obstacles_) {
    int xo1 = obs.center.translation.x-obs.width;
    int xo2 = obs.center.translation.x+obs.width;
    int yo1 = obs.center.translation.y-obs.height;
    int yo2 = obs.center.translation.y+obs.height;
    int xgc1 = gc.center.translation.x-CELL_WIDTH;
    int xgc2 = gc.center.translation.x+CELL_WIDTH;
    int ygc1 = gc.center.translation.y-CELL_HEIGHT;
    int ygc2 = gc.center.translation.y+CELL_HEIGHT;

    if ((xgc1 > xo1 && xgc1 < xo2) || (xgc2 > xo1 && xgc2 < xo2)) { // if left or right bound betw obst bounds
      if ((ygc1 > yo1 && ygc1 < yo2) || (ygc2 > yo1 && ygc2 < yo2)) { // AND if top or bottom bound betw obst bounds 
        return true;
      }
    }
  }
  return false;
}
