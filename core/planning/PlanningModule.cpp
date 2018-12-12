#include <planning/PlanningModule.h>
#include <memory/PlanningBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>
#include <iostream>

PlanningModule::PlanningModule() : tlogger_(textlogger) {
  //std::cout << "grid.size(): " << grid().size() << std::endl;
  GG_ = std::make_unique<GridGenerator>();
  WP_ = std::make_unique<WavefrontPropagation>();
  DSL_ = std::make_unique<CoverageDSL>(cache_, tlogger_);
}

PlanningModule::~PlanningModule() {
  if (initial_cost_map_ == NULL) delete(initial_cost_map_);
}

void PlanningModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("planning");
}

void PlanningModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
  getOrAddMemoryBlock(cache_.planning, "planning");
}

// Perform startup initialization for planning
void PlanningModule::initSpecificModule() {
  grid().resize(GRID_SIZE);
  initial_cost_map_ = new Grid(grid());
  startLoc_ = Point2D(cache_.planning->startPoint.x, cache_.planning->startPoint.y);
  prevLoc_r = getGridRow(startLoc_.x);
  prevLoc_c = getGridCol(startLoc_.y);
  //Pose2D wfStartPose;

  wfStartPose.translation.x = startLoc_.x + 1500; //mm
  wfStartPose.translation.y = 1250 - startLoc_.y; //mm
  wfStartPose.rotation = 0;
  printf("Generating grid\n");
  GG_->generateGrid(*initial_cost_map_);
  printf("Generating wave\n");
  WP_->getCosts(*initial_cost_map_, wfStartPose);
  
  DSL_->init(initial_cost_map_->cells, WP_->start_index_);

  std::cout << "Initialized D* lite. Initial path: " << std::endl;

  for (int i = 0; i < cache_.planning->nodesInPath; ++i) {
    if (i % 10 == 0) {
      std::cout << std::endl;
    }
    
    std::cout << "(" << getRowFromIdx(cache_.planning->path.at(i)) << ", " << getColFromIdx(cache_.planning->path.at(i)) << ") ";
  }
  std::cout << std::endl;

  int desiredCellIdx = cache_.planning->path[cache_.planning->pathIdx];

  // std::cout << "pathIdx: " << cache_.planning->pathIdx << ". I should be in idx " << desiredCellIdx << "  (" << getRowFromIdx(desiredCellIdx)  <<
  //     ", " << getColFromIdx(desiredCellIdx) << ")." << std::endl;

  // TODO: re-initialize features in planning block (maybe shift planning stuff to world_object)?
}

void PlanningModule::processFrame() {
  if (cache_.planning->resetPath) {
    if (initial_cost_map_ == NULL) delete(initial_cost_map_);

    // Make a new initial_cost_map_
    initial_cost_map_ = new Grid(grid());
    
    // Mark all cells as unvisited
    for (int i = 0; i < GRID_SIZE; ++i) {
      initial_cost_map_->cells.at(i).visited = false;
    }
    GG_->generateGrid(*initial_cost_map_);
    WP_->getCosts(*initial_cost_map_, wfStartPose);
    DSL_->init(initial_cost_map_->cells, WP_->start_index_);
    cache_.planning->resetPath = false;
    std::cout << "Re-initialized planning" << std::endl;
    return;
  }
  
  updateCell();
  
  
  // Check if any edge costs have changed
  if (!cache_.planning->changedCost) return;

  DSL_->runDSL();
}

// Check if robot has moved to new cell
// If so, visit the current cell
// If this is the correct cell in the path, increment the pathIdx
// Otherwise, warn the user and maintain the same cell destination
void PlanningModule::updateCell() {
  //std::cout << "\n\n\n\n\ndoes this happen?\n\n\n\n\n" << std::endl;
  
  // check in planning block whether coverage has started
  if (!cache_.planning->coverageStarted) return;
  
  auto& robot = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  int curr_r = getGridRow(robot.loc.y);
  int curr_c = getGridCol(robot.loc.x);

  // if we haven't changed cells, return
  //if (cache_.planning->pathIdx != 0 && curr_r == prevLoc_r && curr_c == prevLoc_c) return;
  // TODO: check if the cell we're trying to go to is occupied??

  int desiredCellIdx = cache_.planning->path[cache_.planning->pathIdx];
  int currIdx = getCellIdx(curr_r, curr_c);

  // get information about the new cell
  if (currIdx != desiredCellIdx) {
    // std::cout << "I should be in idx " << desiredCellIdx << "  (" << getRowFromIdx(desiredCellIdx)  <<
    //   ", " << getColFromIdx(desiredCellIdx) << "), but I'm in " << currIdx << " (" <<
    //   curr_r << ", " << curr_c << ")." << std::endl;

    // TODO: Check if desired cell is occupied and trigger replanning?

    // std::cout << "rest of path: " << std::endl;
    // for (int i = cache_.planning->pathIdx; i < cache_.planning->nodesLeft; ++i) {
      // if (i % 10 == 0)
        // std::cout << std::endl;

      // std::cout << "(" << getRowFromIdx(cache_.planning->path.at(i)) << ", " << getColFromIdx(cache_.planning->path.at(i)) << ") ";
    // }
    // std::cout << std::endl;
   
    return;
  }
  //std::cout << "Marking cell " << getRowFromIdx(currIdx) << ", " << getColFromIdx(currIdx) << " visited" << std::endl;
  cache_.planning->grid.at(currIdx).visited = true;

  // Remove a node on the path
  cache_.planning->nodesLeft--;
  
  // Increment the place along the path
  cache_.planning->pathIdx++;
}
