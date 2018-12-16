#include <planning/GridGenerator.h>

GridGenerator::GridGenerator() {
}

bool GridGenerator::generateGrid(Grid& grid, bool gridCellsGlobalized, bool leaveOccupied) {
	for (auto& cell : grid.cells) {
    if (gridCellsGlobalized) {
      cell.center.translation.x = cell.center.translation.x + FIELD_WIDTH/2.0;
      cell.center.translation.y = -cell.center.translation.y + FIELD_HEIGHT/2.0;
      cell.center.rotation = 0;
    }

		if (!leaveOccupied) cell.occupied = checkObstruction(cell);
		
		// printf("Processing cell at [%d,%d]. Occupied: %d\n",cell.r,cell.c, cell.occupied);
		if (cell.occupied) {
			//std::cout << "cell " << cell.r << ", " << cell.c << " occupied??" << std::endl;
		}
	}
	return true;
}

bool GridGenerator::checkObstruction(GridCell& gc) {
	for (auto& obs : obstacles_) {
		int xo1 = obs.center.translation.x-obs.width;
		int xo2 = obs.center.translation.x+obs.width;
		int yo1 = obs.center.translation.y-obs.height;
		int yo2 = obs.center.translation.y+obs.height;
		int xgc1 = gc.center.translation.x-float(CELL_WIDTH)/2.0;
		int xgc2 = gc.center.translation.x+float(CELL_WIDTH)/2.0;
		int ygc1 = gc.center.translation.y-float(CELL_HEIGHT)/2.0;
		int ygc2 = gc.center.translation.y+float(CELL_HEIGHT)/2.0;

		// printf("Obstacle bounds:\n[%d,%d]\n[%d,%d]\n",xo1,yo1,xo2,yo2);
		// printf("Cell bounds:\n[%d,%d]\n[%d,%d]\n",xgc1,ygc1,xgc2,ygc2);


		if ((xo1 > xgc1 && xo1 < xgc2) || (xo2 > xgc1 && xo2 < xgc2)) { // if left or right bound betw obst bounds
			if ((yo1 > ygc1 && yo1 < ygc2) || (yo2 > ygc1 && yo2 < ygc2)) { // AND if top or bottom bound betw obst bounds 
				// printf("Cell at [%d,%d] is occupied\n",gc.r,gc.c);
				return true;
			}
		}
	}
	return false;
}
