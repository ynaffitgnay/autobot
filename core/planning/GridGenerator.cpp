#include <planning/GridGenerator.h>

GridGenerator::GridGenerator(MemoryCache& cache) : cache_(cache) {
  auto& obs1 = cache_.world_object->objects_[WO_OBSTACLE_1];
  auto& obs2 = cache_.world_object->objects_[WO_OBSTACLE_2];
  obs1.width = 600.0;
  obs1.length = 300.0;
	obs2.width = 600.0;
  obs2.length = 300.0;
}

bool GridGenerator::generateGrid(Grid& grid) {
	for (auto& cell : grid.cells) {
		cell.occupied = checkObstruction(cell);
		// printf("Processing cell at [%d,%d]. Occupied: %d\n",cell.r,cell.c, cell.occupied);
	}
	return true;
}

bool GridGenerator::checkObstruction(GridCell& gc) {
  auto& obs1 = cache_.world_object->objects_[WO_OBSTACLE_1];
  auto& obs2 = cache_.world_object->objects_[WO_OBSTACLE_2];

	int xo11 = obstacles_[WO_OBSTACLE_1].translation.x-obs1.width;
	int xo21 = obstacles_[WO_OBSTACLE_1].translation.x+obs1.width;
	int yo11 = obstacles_[WO_OBSTACLE_1].translation.y-obs1.length;
	int yo21 = obstacles_[WO_OBSTACLE_1].translation.y+obs1.length;
	int xgc1 = gc.center.translation.x-float(CELL_WIDTH)/2.0;
	int xgc2 = gc.center.translation.x+float(CELL_WIDTH)/2.0;
	int ygc1 = gc.center.translation.y-float(CELL_HEIGHT)/2.0;
	int ygc2 = gc.center.translation.y+float(CELL_HEIGHT)/2.0;

	int xo12 = obstacles_[WO_OBSTACLE_2].translation.x-obs2.width;
	int xo22 = obstacles_[WO_OBSTACLE_2].translation.x+obs2.width;
	int yo12 = obstacles_[WO_OBSTACLE_2].translation.y-obs2.length;
	int yo22 = obstacles_[WO_OBSTACLE_2].translation.y+obs2.length;

	if ((xo11 > xgc1 && xo11 < xgc2) || (xo21 > xgc1 && xo21 < xgc2)) { // if left or right bound betw obst bounds
		if ((yo11 > ygc1 && yo11 < ygc2) || (yo21 > ygc1 && yo21 < ygc2)) { // AND if top or bottom bound betw obst bounds 
			// printf("Cell at [%d,%d] is occupied\n",gc.r,gc.c);
			return true;
		}
	}

	if ((xo12 > xgc1 && xo12 < xgc2) || (xo22 > xgc1 && xo22 < xgc2)) { // if left or right bound betw obst bounds
		if ((yo12 > ygc1 && yo12 < ygc2) || (yo22 > ygc1 && yo22 < ygc2)) { // AND if top or bottom bound betw obst bounds 
			// printf("Cell at [%d,%d] is occupied\n",gc.r,gc.c);
			return true;
		}
	}
	return false;
}