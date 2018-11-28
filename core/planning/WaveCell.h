#ifndef WAVE_CELL_H_
#define WAVE_CELL_H_

#include <math/Point.h>
#include <planning/structures/GridCell.h>

static const int WAVE_END = 0;           // wavefront value used for end cell (must be zero because wave starts propogating upward from here)
static const int WAVE_OBSTRUCTION = -1;  // wavefront value used for obstruction 
static const int WAVE_INIT = -2;         // wavefront value used fot initialization
static const int WAVE_START = -3;        // wavefront value used for start cell 

static const int NUM_NEIGHBORS = 4;      // number of connected neighbors

class WaveCell {
private:
  int wave_value_;
  bool occupied_;
  bool is_labelled_;
  bool is_initialized_;
  Point position_;
  std::vector<WaveCell*> neighbors_
  GridCell gc_;
  setNeighbors();
  void setValue();


public:
  WaveCell();
  ~WaveCell();
  WaveCell(int wave_value, bool occupied, bool isLabelled, bool isInitialized, Point position, std::vector<WaveCell*> neighbors, GridCell gc);:
    wave_value_(WAVE_INIT), occupied_(false), is_labelled_(false), is_initialized_(false), position_(position), neighbors_(NUM_NEIGHBORS,NULL), gc_(gc) { }
  bool contains(Point pose);
  int getValue();
  int setNeighborsValues(int wave_label);

};

#endif // WAVE_CELL_H_


