#ifndef GRID_GENERATOR_H_
#define GRID_GENERATOR_H_

#include <memory/MemoryCache.h>
#include <planning/structures/GridCell.h>
#include <planning/structures/Grid.h>
#include <planning/structures/Obstacle.h>
#include <planning/PlanningConstants.h>
#include <memory/WorldObjectBlock.h>
#include <memory/WorldObject.h>
#include <math/Pose2D.h>


class GridGenerator {
private:
  // std::vector<Obstacle> obstacles_= { Obstacle(Pose2D(0.0, 1200.0, 1500.0), 30, 60), 
  //                                     Obstacle(Pose2D(0.0, 2000.0, 600.0), 30, 60) };
  MemoryCache cache_;
  
  bool checkObstruction(GridCell& cell);

public:
  GridGenerator(MemoryCache& cache);
  bool generateGrid(Grid& map);
  std::map<WorldObjectType,Pose2D> obstacles_ = {
    { WO_OBSTACLE_1, Pose2D(0.0, 1800.0, 750.0) },
    { WO_OBSTACLE_2, Pose2D(0.0, 750.0, 1350.0) }};

};

#endif // GRID_GENERATOR_H_
