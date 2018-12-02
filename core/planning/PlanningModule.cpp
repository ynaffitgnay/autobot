#include <planning/PlanningModule.h>
#include <memory/PlanningBlock.h>

PlanningModule::PlanningModule() : tlogger_(textlogger) {
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
  std::cout << "cache_.planning->startPoint.x: " << cache_.planning->startPoint.x << std::endl;
  startLoc_ = Point2D(cache_.planning->startPoint.x, cache_.planning->startPoint.y);
  Pose2D wfStartPose;

  wfStartPose.translation.x = startLoc_.x + 1500; //mm
  wfStartPose.translation.y = 1250 - startLoc_.y; //mm
  wfStartPose.rotation = 0;
  printf("Generating grid\n");
  GG_->generateGrid(initial_cost_map_);
  printf("Generating wave\n");
  WP_->getCosts(initial_cost_map_, wfStartPose);
  DSL_->init(initial_cost_map_);

  // TODO: re-initialize features in planning block (maybe shift planning stuff to world_object)?
}

void PlanningModule::processFrame() {
  visitNewCell();
  // Check if any obstacles have been encountered -- maybe store this in world objects?
  DSL_->runDSL();
}

void PlanningModule::visitNewCell() {
  // check in planning block whether coverage has started
  // then visit a new cell
}
