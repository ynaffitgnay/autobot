#include <memory/PlanningBlock.h>

PlanningBlock::PlanningBlock() {
  header.version = 1;
  header.size = sizeof(PlanningBlock);
  grid = std::vector<PathNode>(GRID_SIZE);
  changedCost = false;
  coverageStarted = false;
  pathIdx = 0;
  path = std::vector<PathNode>(PATH_SIZE);
}
