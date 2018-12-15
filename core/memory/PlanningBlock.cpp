#include <memory/PlanningBlock.h>

PlanningBlock::PlanningBlock() {
  header.version = 1;
  header.size = sizeof(PlanningBlock);
  startPoint = Point2D(START_X, START_Y);
  changedCost = false;
  coverageStarted = false;
  observedNextGC = false;
  resetPath = false;
  pathIdx = 0;
  path = std::vector<int>(PATH_SIZE);
  nodesLeft = 0;  // to keep track of when finished
  nodesInPath = 0;
  pathsPlanned = 0;

  for (int r = 0; r < GRID_HEIGHT; ++r) {
    for (int c = 0; c < GRID_WIDTH; ++c) {
      grid.push_back(GridCell(r,c));
    }
  }
}

void PlanningBlock::RestartPath() {
  coverageStarted = false;
  observedNextGC = false;
  resetPath = true;
  pathIdx = 0;
  nodesLeft = nodesInPath;
}

Pose2D PlanningBlock::getDestPose() {
  return grid.at(path.at(pathIdx)).center;
}

int PlanningBlock::getDestGridRow() {
  return getRowFromIdx(path.at(pathIdx));
}

int PlanningBlock::getDestGridCol() {
  return getColFromIdx(path.at(pathIdx));
}

int PlanningBlock::getGridRowFromLoc(float y) {
  return getGridRow(y);
}   
    
int PlanningBlock::getGridColFromLoc(float x) {
  return getGridCol(x);
}
