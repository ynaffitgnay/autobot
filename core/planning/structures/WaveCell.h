#ifndef WAVE_CELL_H_
#define WAVE_CELL_H_

#include <math/Point.h>
#include <planning/structures/GridCell.h>
#include <planning/PlanningConstants.h>
#include <vector>
#include <algorithm>

static const int WAVE_END = 0;           // wavefront value used for end cell (must be zero because wave starts propogating upward from here)
static const int WAVE_OBSTRUCTION = -1;  // wavefront value used for obstruction 
static const int WAVE_INIT = -2;         // wavefront value used fot initialization
static const int WAVE_START = -3;        // wavefront value used for start cell 

static const int NUM_NEIGHBORS = 4;      // number of connected neighbors

class WaveCell {
private:
  int wave_value_;
  bool occupied_;
  bool is_planned_;
  bool is_initialized_;
  Pose2D position_;
  std::vector<WaveCell*> neighbors_;
  GridCell gc_;


public:
  WaveCell();
  ~WaveCell();
  WaveCell(Pose2D position, GridCell gc); 
  bool contains(Pose2D pose);
  int getValue(void);
  int setNeighborsValues(int wave_label);
  int getLandmarkFactor(void);
  std::vector<WaveCell*> getNeighbors(void);
  Pose2D getPosition(void);
  void setNeighbors(std::vector<WaveCell*> nbs);
  bool setValue(int value);
  std::vector<WaveCell*> getUnvisitedNeighbors(void);
  void visit(void);
  bool isVisited(void);

};

#endif // WAVE_CELL_H_


