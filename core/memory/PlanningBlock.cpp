#include <memory/PlanningBlock.h>

PlanningBlock::PlanningBlock() {
  header.version = 1;
  header.size = sizeof(PlanningBlock);
  // state = decltype(state)::Zero();
  //covariance = decltype(covariance)::Identity();
  grid = std::vector<PathNode>(GRID_SIZE);
}
