#include <memory/PlanningBlock.h>

PlanningBlock::PlanningBlock() {
  header.version = 1;
  header.size = sizeof(PlanningBlock);
  startPoint = Point2D(START_X, START_Y);
  //grid = std::vector<GridCell>(GRID_SIZE);
  changedCost = false;
  coverageStarted = false;
  pathIdx = 0;
  path = std::vector<GridCell>(PATH_SIZE);

  for (int r = 0; r < GRID_HEIGHT; ++r) {
    for (int c = 0; c < GRID_WIDTH; ++c) {
      grid.push_back(GridCell(r,c));
    }
  }
}

Pose2D PlanningBlock::getDestPose() {
  return path.at(pathIdx).center;
}
