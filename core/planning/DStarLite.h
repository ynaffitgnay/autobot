#ifndef _D_STAR_LITE_H
#define _D_STAR_LITE_H

#include <common/PathNode.h>
#include <planning/Grid.h>
#include <queue>

// TODO: MOVE THESE #DEFINES
#define GRID_HEIGHT (300)
#define GRID_WIDTH  (200)

class DStarLite
{
public:
  DStarLite();
  std::priority_queue<PathNode> U_;
  int k_;  // key modifier

  void init(Grid& wavefront);
  void runDSL();
private:
  int calcKey(PathNode successor);
  void updateVertex(PathNode u);
  void computeShortestPath();
}

//TODO: this should not build yet bc end of class doesn't have a fuckin semicolon

#endif
