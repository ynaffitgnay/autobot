#include <planning/PlanningModule.h>
#include <memory/PlanningBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>
#include <iostream>

PlanningModule::PlanningModule() : tlogger_(textlogger), AStar_(false) {
  //std::cout << "grid.size(): " << grid().size() << std::endl;
  GG_ = std::make_unique<GridGenerator>();
  WP_ = std::make_unique<WavefrontPropagation>();
  DSL_ = std::make_unique<CoverageDSL>(cache_, tlogger_);
}

PlanningModule::~PlanningModule() {
  if (initial_cost_map_ != NULL) delete(initial_cost_map_);
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
  GG_->generateGrid(*initial_cost_map_, false);
  printf("Generating wave\n");
  WP_->getCosts(*initial_cost_map_, wfStartPose, !AStar_);
  
  DSL_->init(initial_cost_map_->cells, WP_->start_index_, AStar_);

  std::cout << "Initialized D* lite. Initial path: " << std::endl;

  for (int i = 0; i < cache_.planning->nodesInPath; ++i) {
    if (i % 10 == 0) {
      std::cout << std::endl;
    }
    
    std::cout << "(" << getRowFromIdx(cache_.planning->path.at(i)) << ", " << getColFromIdx(cache_.planning->path.at(i)) << ") ";
  }
  std::cout << std::endl;
}

void PlanningModule::processFrame() {
  if (cache_.planning->resetPath) {
    if (initial_cost_map_ != NULL) delete(initial_cost_map_);

    // Make a new initial_cost_map_
    initial_cost_map_ = new Grid(grid());
    
    // Mark all cells as unvisited
    for (int i = 0; i < GRID_SIZE; ++i) {
      initial_cost_map_->cells.at(i).visited = false;
    }
    GG_->generateGrid(*initial_cost_map_, true);
    WP_->getCosts(*initial_cost_map_, wfStartPose, !AStar_);
    DSL_->init(initial_cost_map_->cells, WP_->start_index_);
    cache_.planning->resetPath = false;
    std::cout << "Re-initialized planning" << std::endl;
    return;
  }
  
  updateCell();
  
  // Check if any edge costs have changed
  if (!cache_.planning->changedCost) return;

  if (!AStar_) {
    DSL_->runDSL();
  } else {
    // Mark the most recent cell as occupied
    initial_cost_map_->cells.at(cache_.planning->path.at(cache_.planning->pathIdx)).occupied = true;
    wfStartPose = cache_.planning->grid.at(cache_.planning->path.at(cache_.planning->pathIdx - 1)).center;
    wfStartPose.translation.x = wfStartPose.translation.x + FIELD_WIDTH/2.0;
    wfStartPose.translation.y = -wfStartPose.translation.y + FIELD_HEIGHT/2.0;
    wfStartPose.rotation = 0;
    GG_->generateGrid(*initial_cost_map_, true, true);
    WP_->getCosts(*initial_cost_map_, wfStartPose, !AStar_, cache_.planning->path.at(cache_.planning->pathIdx - 1));
    DSL_->init(initial_cost_map_->cells, WP_->start_index_, true);
    cache_.planning->changedCost = false;

    std::cout << "Prev path followed:" << std::endl;
    for (int i = 0; i < cache_.planning->pathIdx; ++i) {
      if (i % 10 == 0) {
        std::cout << std::endl;
      }  
      std::cout << "(" << getRowFromIdx(cache_.planning->path.at(i)) << ", " << getColFromIdx(cache_.planning->path.at(i)) << ") ";
    }
    std::cout << std::endl;
  }
  
  std::cout << "Replanned beginning with pathIdx " << cache_.planning->pathIdx << std::endl;
  
  for (int i = cache_.planning->pathIdx; i < cache_.planning->nodesInPath; ++i) {
    if (i % 10 == 0) {
      std::cout << std::endl;
    }
    
    std::cout << "(" << getRowFromIdx(cache_.planning->path.at(i)) << ", " << getColFromIdx(cache_.planning->path.at(i)) << ") ";
  }
  std::cout << std::endl;
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

  int desiredCellIdx = cache_.planning->path[cache_.planning->pathIdx];
  int currIdx = getCellIdx(curr_r, curr_c);

  // get information about the new cell
  if (currIdx != desiredCellIdx) return;

  cache_.planning->grid.at(currIdx).visited = true;

  // Remove a node on the path
  cache_.planning->nodesLeft--;
  
  // Increment the place along the path
  cache_.planning->pathIdx++;
  std::cout << "Moved to pathIdx " << cache_.planning->pathIdx << std::endl;
}

void PlanningModule::setAStar() {
  std::cout << "Resetting path planning to use AStar" << std::endl;
  AStar_ = true;
  cache_.planning->reInitPath();
  processFrame();
}

void PlanningModule::setDSL() {
  std::cout << "Resetting path planning to use DStarLite" << std::endl;
  AStar_ = false;
  cache_.planning->reInitPath();
  processFrame();
}

bool PlanningModule::isAStar() {
  return AStar_;
}
