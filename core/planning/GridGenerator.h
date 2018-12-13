#ifndef GRID_GENERATOR_H_
#define GRID_GENERATOR_H_

#include <planning/structures/GridCell.h>
#include <planning/structures/Grid.h>
#include <planning/structures/Obstacle.h>
#include <planning/PlanningConstants.h>
#include <memory/WorldObjectBlock.h>
#include <math/Pose2D.h>


class GridGenerator {
private:
  std::vector<Obstacle> obstacles_= { Obstacle(Pose2D(0.0, 1800.0, 750.0), 30, 60), 
                                      Obstacle(Pose2D(0.0, 900.0, 1350.0), 30, 60) };
  bool checkObstruction(GridCell& cell);

public:
  GridGenerator();
  bool generateGrid(Grid& map, bool gridCellsGlobalized);
  // std::map<WorldObjectType,Pose2D> obstacles_ = {
  //   { WO_OBSTACLE_1, Pose2D(0.0, 1800.0, 750.0) },
  //   { WO_OBSTACLE_2, Pose2D(0.0, 900.0, 1350.0) }};

};

#endif // GRID_GENERATOR_H_
