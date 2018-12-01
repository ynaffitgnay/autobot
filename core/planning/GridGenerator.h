#ifndef GRID_GENERATOR_H_
#define GRID_GENERATOR_H_

#include <math/Pose2D.h>
#include <planning/structures/GridCell.h>
#include <planning/structures/Obstacle.h>


class GridGenerator {
private:
  std::vector<Obstacle> obstacles_= { Obstacle(Pose2D(0.0, 1200.0, 1500.0), 40, 60), 
                                      Obstacle(Pose2D(0.0, 2000.0, 600.0), 40, 60) };
  bool checkObstruction(GridCell& cell);


public:
  GridGenerator();
  ~GridGenerator();
  bool generateGrid(Grid& map);

};

#endif // GRID_GENERATOR_H_
