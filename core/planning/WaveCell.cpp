#include <planning/WaveCell.h>

WaveCell::WaveCell() {
  // Default Constructor
}

WaveCell::~WaveCell() {
  // Default Destructor
}

WaveCell::WaveCell(Point position, GridCell gc):
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

WavefrontCell::~WavefrontCell() {
  // Do nothing
}

void WavefrontCell::setNeighbors(std::vector<WavefrontCell*> nbs) {
  neighbors_ = nbs;
}

std::vector<WavefrontCell*> WavefrontCell::getNeighbors() {
  return neighbors_; 
}

bool WavefrontCell::setValue(int value) {
  // Set the cell to the wave value
  // if the cell has not been changed since initialization or if the value is reinitializing the cell (don't set it if it has already been assigned a wave value)
  // in any case, do not set the value if this cell is an obstruction (this data comes directly from the map and should not be changed)
  if((WAVE_INIT == wave_value_ || WAVE_INIT == value) && WAVE_OBSTRUCTION != wave_value_)
  {
    wave_value_ =  value;
    return true;
  }
  else
  {
    return false;
  }
}

int WavefrontCell::getValue() {   
  return wave_value_;
}

int WavefrontCell::setNeighborsValues(int wave_label) {
  // Set the wave value of this cell's neighbors and return how many were set successfully
  int count = 0;
  for(int i = 0; i<neighbors_.size(); i++)
  {
    if(neighbors_[i]->setCell(wave_label))
      count++;
  }
  return count;
}

Point WavefrontCell::getPosition() {
  return position_;
}

void WavefrontCell::visit() {
  is_labelled_ = true;
}

bool WavefrontCell::isVisited() {
  return is_labelled_;
}

void WavefrontCell::reset() {
  is_labelled_ = false;
  
  if(occupied_)
    wave_value_ = WAVE_OBSTRUCTION;
  else
    wave_value_ = WAVE_INIT;
}

bool WavefrontCell::contains(Point pose) {
  // If the given pose is within the cell boundaries, return true
   // ROS_INFO("PoseY: %5.5f Vertex: %5.5f",pose.pose.position.y , mapCell.vertices.points[0].y);
   // ROS_INFO("       PoseX: %5.5f   VertexX: %5.5f",pose.pose.position.x , mapCell.vertices.points[0].x);
  if(pose.pose.position.x > mapCell.vertices.points[0].x &&  //top left
     pose.pose.position.x < mapCell.vertices.points[1].x &&  //top right
     pose.pose.position.y > mapCell.vertices.points[2].y &&  //bottom left
     pose.pose.position.y < mapCell.vertices.points[0].y)    //top left
  {
    return true;
  }
  else
  {
    return false;
  }
}


