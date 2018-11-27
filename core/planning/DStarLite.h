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
  DStarLite();
  std::priority_queue<PathNode> U_;
  int k_; // key modifier

  void init(Grid& wavefront);
  
  int calcKey(PathNode s);
}

#endif
