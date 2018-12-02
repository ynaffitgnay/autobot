#include <memory/PlanningBlock.h>

PlanningBlock::PlanningBlock() {
  header.version = 1;
  header.size = sizeof(PlanningBlock);
  startPoint = Point2D(START_X, START_Y);
  grid = std::vector<PathNode>(GRID_SIZE);
  changedCost = false;
  coverageStarted = false;
  pathIdx = 0;
  path = std::vector<PathNode>(PATH_SIZE);
}
