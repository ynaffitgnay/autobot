#include <planning/GridGenerator.h>

GridGenerator::GridGenerator() {
  // Default Constructor
}

GridGenerator::~GridGenerator() {
  // Default Destructor
}

bool GridGenerator::generateGrid(Grid& grid) {
	for (auto& cell : grid) {
		cell.occupied = checkObstruction(cell);
	}
}

bool GridGenerator::checkObstruction(GridCell& gc) {
	float x1,x2,y1,y2;
	for (auto& obs : obstacles_) {
		xo1 = obs.center.translation.x-obs.width;
		xo2 = obs.center.translation.x+obs.width;
		yo1 = obs.center.translation.y-obs.height;
		yo2 = obs.center.translation.y+obs.height;
		xgc1 = gc.center.translation.x-CELL_WIDTH;
		xgc2 = gc.center.translation.x+CELL_WIDTH;
		ygc1 = gc.center.translation.y-CELL_HEIGHT;
		ygc2 = gc.center.translation.y+CELL_HEIGHT;

		if ((xgc1 > xo1 && xgc1 < xo2) || (xgc2 > xo1 && xgc2 < xo2)) { // if left or right bound betw obst bounds
			if ((ygc1 > yo1 && ygc1 < yo2) || (ygc2 > yo1 && ygc2 < yo2)) { // AND if top or bottom bound betw obst bounds 
				return true;
			}
		}
	}
}