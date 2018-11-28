#include <planning/PlanningModule.h>

PlanningModule::PlanningModule() : tlogger_(textlogger) {
  //GG_ = std::make_unique<GridGenerator>(cache_);
  //WP_ = std::make_unique<WavefrontPropagation>(cache_);
  DSL_ = std::make_unique<DStarLite>(cache_, tlogger_);
}

void PlanningModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
}

void PlanningModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
}

// Perform startup initialization for planning
void PlanningModule::initSpecificModule() {
  // TODO: get the wavefront costs in a grid here
  //GG_->generateGrid(initial_cost_map_);
  //WP_->getCosts(initial_cost_map_, startPose);
  DSL_->init(initial_cost_map_);

  // TODO: re-initialize features in planning block (maybe shift planning stuff to world_object)?
}

void PlanningModule::processFrame() {
  // Check if any obstacles have been encountered -- maybe store this in world objects?
  DSL_->runDSL();
}
