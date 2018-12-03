#include <planning/PlanningModule.h>
#include <memory/PlanningBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>
#include <iostream>

PlanningModule::PlanningModule() : tlogger_(textlogger) {
  //std::cout << "grid.size(): " << grid().size() << std::endl;
  GG_ = std::make_unique<GridGenerator>();
  WP_ = std::make_unique<WavefrontPropagation>();
  DSL_ = std::make_unique<DStarLite>(cache_, tlogger_, Point2D(START_X, START_Y));
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
  Grid initial_cost_map = Grid(grid());
  //std::cout << "grid.size() 2: " << grid().size() << std::endl;
  ///std::cout << "initial_cost_map.cells.size(): " << initial_cost_map.cells.size() << std::endl;
  startLoc_ = Point2D(cache_.planning->startPoint.x, cache_.planning->startPoint.y);
  prevLoc_r = getGridRow(startLoc_.x);
  prevLoc_c = getGridCol(startLoc_.y);
  Pose2D wfStartPose;

  wfStartPose.translation.x = startLoc_.x + 1500; //mm
  wfStartPose.translation.y = 1250 - startLoc_.y; //mm
  wfStartPose.rotation = 0;
  printf("Generating grid\n");
  GG_->generateGrid(initial_cost_map);
  printf("Generating wave\n");
  WP_->getCosts(initial_cost_map, wfStartPose);
  DSL_->init(initial_cost_map);

  std::cout << "Initialized D* lite" << std::endl;

  // TODO: re-initialize features in planning block (maybe shift planning stuff to world_object)?
}

void PlanningModule::processFrame() {
  updateCell();
  // Check if any obstacles have been encountered -- maybe store this in world objects?
  DSL_->runDSL();
}

// Check if robot has moved to new cell
// If so, visit the current cell
// If this is the correct cell in the path, increment the pathIdx
// Otherwise, warn the user and maintain the same cell destination
void PlanningModule::updateCell() {
  // check in planning block whether coverage has started
  if (!cache_.planning->coverageStarted) return;
  
  auto& robot = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  int curr_r = getGridRow(robot.loc.y);
  int curr_c = getGridCol(robot.loc.x);

  // if we haven't changed cells, return
  if (curr_r == prevLoc_r && curr_c == prevLoc_c) return;

  int desiredCellIdx = cache_.planning->path[cache_.planning->pathIdx];
  int currIdx = getCellIdx(curr_r, curr_c);

  // get information about the new cell
  if (currIdx != desiredCellIdx) { //cache_.planning->path[cache_.planning->pathIdx]) {
    std::cout << "I'm in the wrong gridCell!!" << std::endl;
    std::cout << "I should be in " << desiredCellIdx << ", but I'm in " << currIdx << std::endl;
    std::cout << "I guess I'm going to keep trying to go to my original destination" << std::endl;
    return;
  }

  // Remove a node on the path
  cache_.planning->nodesLeft--;
  
  // Increment the place along the path
  cache_.planning->pathIdx++;

  // TODO:
  

  //if (currIdx == 0) 
}
