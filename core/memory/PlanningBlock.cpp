#include <memory/PlanningBlock.h>

PlanningBlock::PlanningBlock() {
  header.version = 1;
  header.size = sizeof(PlanningBlock);
  startPoint = Point2D(START_X, START_Y);
  //grid = std::vector<GridCell>(GRID_SIZE);
  changedCost = false;
  coverageStarted = false;
  resetPath = false;
  pathIdx = 0;
  //path = std::vector<GridCell>(PATH_SIZE);
  path = std::vector<int>(PATH_SIZE);
  nodesLeft = 0;  // to keep track of when finished
  nodesInPath = 0;

  for (int r = 0; r < GRID_HEIGHT; ++r) {
    for (int c = 0; c < GRID_WIDTH; ++c) {
      grid.push_back(GridCell(r,c));
    }
  }
}

void PlanningBlock::RestartPath() {
  coverageStarted = false;
  resetPath = true;
  pathIdx = 0;
  nodesLeft = nodesInPath;
}

Pose2D PlanningBlock::getDestPose() {
  return grid.at(path.at(pathIdx)).center;
}

