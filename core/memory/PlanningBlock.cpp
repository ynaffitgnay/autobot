#include <memory/PlanningBlock.h>

PlanningBlock::PlanningBlock() {
  header.version = 1;
  header.size = sizeof(PlanningBlock);
  grid = std::vector<PathNode>(GRID_SIZE);
  path_idx = 0;
  path = std::vector<PathNode>(PATH_SIZE);
}
