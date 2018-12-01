#include <planning/WaveCell.h>

WaveCell::WaveCell() {
  // Default Constructor
}

WaveCell::~WaveCell() {
  // Default Destructor
}

WaveCell::WaveCell(Pose2D position, GridCell gc):
    wave_value_(WAVE_INIT), occupied_(false), is_labelled_(false), is_initialized_(false), position_(position), neighbors_(NUM_NEIGHBORS,NULL), gc_(gc) { 
    if(gc_.occupied) {
      wave_value_ = WAVE_OBSTRUCTION;
      occupied_ = true;
    }
    else{
      wave_value_ = WAVE_INIT;
    }
  
  is_initialized_ = true;
}

void WaveCell::setNeighbors(std::vector<WaveCell*> nbs) {
  neighbors_ = nbs;
}

std::vector<WaveCell*> WaveCell::getNeighbors() {
  return neighbors_; 
}

bool WaveCell::setValue(int value) {
  // Set the cell to the wave value
  // if the cell has not been changed since initialization or if the value is reinitializing the cell (don't set it if it has already been assigned a wave value)
  // in any case, do not set the value if this cell is an obstruction (this data comes directly from the map and should not be changed)
  if(WAVE_OBSTRUCTION != wave_value_)
  {
    wave_value_ =  value;
    return true;
  }
  else
  {
    return false;
  }
}

int WaveCell::getValue() {   
  return wave_value_;
}

int WaveCell::setNeighborsValues(int wave_label) {
  // Set the wave value of this cell's neighbors and return how many were set successfully
  int count = 0;
  for(int i = 0; i<neighbors_.size(); i++) {
    if (neighbors_[i]->getValue() == WAVE_INIT) {
      if(neighbors_[i]->setValue(wave_label)) {
        count++;
      }
    }
  }
  return count;
}

int  WaveCell::getWallFactor() {
  int wall_factor;
  int row = gc_.r+1;
  int col = gc_.c+1;

  std::vector<int> wall_dists = {row, GRID_HEIGHT-row, col, GRID_WIDTH-col};
  std::vector<int>::iterator result = std::min_element(std::begin(wall_dists), std::end(wall_dists));
  int min_index = std::distance(std::begin(wall_dists), result);

  switch (min_index) {
    case 0:
      wall_factor = std::round(GRID_HEIGHT/2)-(row-1);
      break;
    case 1:
      wall_factor = std::round(GRID_HEIGHT/2)-( GRID_HEIGHT-row);
      break;
    case 2:
      wall_factor = std::round(GRID_WIDTH/2)-(col-1);
      break;
    case 3:
      wall_factor = std::round(GRID_WIDTH/2)-(GRID_WIDTH-col);
      break;
    default:
      wall_factor = 0;
      break;
  }

  return wall_factor;
}

Pose2D WaveCell::getPosition() {
  return position_;
}


bool WaveCell::contains(Pose2D pose) {
  // If the given pose is within the cell boundaries, return true
  int xwc1 = gc_.center.translation.x-CELL_WIDTH/2;
  int xwc2 = gc_.center.translation.x+CELL_WIDTH/2;
  int ywc1 = gc_.center.translation.y-CELL_HEIGHT/2;
  int ywc2 = gc_.center.translation.y+CELL_HEIGHT/2;
  int poseX = pose.translation.x;
  int poseY = pose.translation.y;

  if ((poseX > xwc1 && poseX < xwc2) && (poseY > ywc1 && poseY < ywc2)) { // if pose within cell bounds 
      return true;
  }
  else
  {
    return false;
  }
}


