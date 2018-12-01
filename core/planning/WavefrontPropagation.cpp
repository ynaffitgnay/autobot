#include "WavefrontPropagation.h"


WavefrontPropagation::WavefrontPropagation() :
  mapSize(0),
  numRows(0),
  numCols(0),
  startIndex(0),
  endIndex(0),
  hasMap(false),
  isInitialized(false)
{
  
}

WavefrontPropagation::~WavefrontPropagation() {
  // Do nothing
}

bool WavefrontPropagation::getCosts(Grid& map, Pose2D& startPose) {
  // Set map variables
  mapSize = map.cells.size();
  numRows = std::round(map.height/map.cell_height);
  numCols = std::round(map.width/map.cell_width);
  
  // Check for invalid parameters
  if(numCols <= 0)
  {
    printf("Grid must have at least one column.\n");
    return false;
  }
  else if(numRows <= 0)
  {
    printf("Grid must have at least one row.\n");
    return false;
  }

  // Initialize wave cells
  waveCells.clear();
  waveCells.resize(map.cells.size());
  for(int i = 0; i<waveCells.size(); i++) {
  	WaveCell wc(map.cells[i].center, map.cells[i]);
    waveCells[i]=wc;
  }
  hasMap = true;
  
  // Figure out which cells are neighbors
  findNeighbors();
  
  // Set init flag
  isInitialized = true;

  if (fill(startPose)) {
  	printf("Success!\n");
    for(int i = 0; i<waveCells.size(); i++) {
      map.cells[i].cost = waveCells[i].getValue();
    }
  	return true;
  } else {
  	printf("Failure! :(\n");
  	return false;
  }
}

bool WavefrontPropagation::fill(Pose2D startPose) {
  // Set start state if it is valid
  if(!setStartIndex(startPose)) {
    return false;
  }
  // Set start and end cell values
  waveCells[startIndex].setValue(WAVE_START);
  waveCells[endIndex].setValue(WAVE_END);
  
  // Fill remaining cells using wavefront propogation
  int fillCount = 2;      // Number of cells filled with wave numbers (not including obstructions)
  int waveNumber = 1;     // Number indicating the current wave
  int lastFillCount = 0;  // Number of cells filled with wave numbers in previous iteration
  while(fillCount != lastFillCount) {
    lastFillCount = fillCount;
    for(auto& wc : waveCells) {
      if(wc.getValue() == waveNumber-1) {
        fillCount += wc.setNeighborsValues(waveNumber);
      }
    }
    waveNumber++;
  }

  // Add a factor for the distance from the wall. We want cells on the outside to have a higher value.
  int alpha = 1;
  std::vector<int> wall_factors;
  for (auto& wc : waveCells) {

    int wallFactor = wc.getWallFactor();
    wall_factors.push_back(wallFactor);
    int newValue = wc.getValue() + alpha*wallFactor;
    // printf("Old wave value: %d Wall factor: %d New wave value: %d\n", wc.getValue(),wallFactor,newValue);
    if (wc.getValue() != WAVE_OBSTRUCTION && wc.getValue() != WAVE_START && wc.getValue() != WAVE_END) {
      wc.setValue(newValue);
    }
  }
  
  /********************************************************************************/
  /** This code block will print the wavefront. Use only for debugging purposes. **/
  /********************************************************************************/
  int grid_cols = numCols;
  int grid_rows = numRows;
  int index;

  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  GridCell* cell;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      index = row*grid_cols+col; // Finding the appropriate cell in vectorized form
      if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
        std::cout<<color_red<< "| |" <<default_col<<"|";
      else
      {
        if(waveCells[index].getValue()<10 && waveCells[index].getValue()>=0)
          std::cout << "| "<< waveCells[index].getValue() <<"|"; 
        else
          std::cout << "|"<< waveCells[index].getValue() <<"|"; 
      }
    }
    std::cout << std::endl;
  }
  std::cout << "----------------------------------------------------------------" << std::endl;
  
  return true;
}



bool WavefrontPropagation::setStartIndex(Pose2D startPose) {
  // Find the cell that contains the start index
  int cellIndex = getIndex(startPose);
  printf("Cell Index: %d\n", cellIndex);
  
  // Make sure the cell is valid and not obstructed
  if (checkPose(startPose)) {
    startIndex = cellIndex;
  }
  else {
    return false;
  }

  // Find a valid neighbor to place end index
  std::vector<WaveCell*> startNeighbors = waveCells[startIndex].getNeighbors();
  if(startNeighbors.size() > 0)
  {
    Pose2D endPose = startNeighbors[0]->getPosition();
    endIndex = getIndex(endPose);
    return true;
  }
  else
  {
   printf("Start pose invalid: has no valid neighbors\n");
    return false;
  }
}

bool WavefrontPropagation::checkPose(Pose2D pose) {
  int cell_index = getIndex(pose);
  if(cell_index >= 0 && cell_index < mapSize)
  {
    if(WAVE_OBSTRUCTION == waveCells[cell_index].getValue())
    {
     printf("Start pose invalid: cell is occupied\n");
      return false;
    }
    return true;
  }
  else
  {
   printf("Start pose invalid: not within map bounds.\n");
    return false;
  }
}

void WavefrontPropagation::findNeighbors() {
  // Set up neighbors vectors for all cells
  std::vector<WaveCell*> neighbors; // vector of valid neighbors to be passed to cell
  for(int i = 0; i<waveCells.size(); i++)
  {
    if(WAVE_OBSTRUCTION != waveCells[i].getValue()) // cell does not need neighbors if it is occupied (and therefore inaccessible)
    {
      neighbors.clear();
      int r,c = 0;
      getCoordinate(i, r, c);
      addNeighbor(neighbors,r+1,c); // try adding possible neighbor that sits below this cell
      addNeighbor(neighbors,r-1,c); // try adding possible neighbor that sits above this cell
      addNeighbor(neighbors,r,c+1); // try adding possible neighbor that sits to the right
      addNeighbor(neighbors,r,c-1); // try adding possible neighbor that sits to the left
      waveCells[i].setNeighbors(neighbors); // set the valid neighbors that we found
    }
  }
}

void WavefrontPropagation::addNeighbor(std::vector<WaveCell*> &neighbors, int r, int c) {
  // add a neighbor to the neighbors array if (r,c) is valid neighbor
  int index = 0;
  if(getIndex(index, r, c)) // false if (r,c) is not within map
  {
    if(WAVE_OBSTRUCTION != waveCells[index].getValue()) // do not add obstructed neighbors
    {
      neighbors.push_back(&waveCells[index]);
    }
  }
}

bool WavefrontPropagation::getCoordinate(int index, int &r, int &c) {
  //return the coordinate corresponding to index if it is withing the map
  if(!hasMap)
  {
    printf("Cannot get coordinate before mapdata is set.\n");
    return false;
  }

  if(index<0 || index >= mapSize)
  {
    printf("Index out of range. Cannot get coordinate\n");
    return false;
  }

  c = index % numCols;
  r = (index -c)/numCols;
  return true;
}

bool WavefrontPropagation::getIndex(int &index, int r, int c) {
  // Return the index corresponding to (r,c) if it is in the map
  if(!hasMap)
  {
    printf("Cannot get index before map data is set.\n");
    return false;
  }

  if(r < 0 || c < 0 || r >= numRows || c >= numCols)
  {
    return false;
  }
  else
  {
    index = r*numCols + c;
    return true;
  }
}

int WavefrontPropagation::getIndex(Pose2D pose) {
  // Return the index of the cell that contains pose
  for(int i = 0; i<waveCells.size(); i++)
  {
    if(waveCells[i].contains(pose)) {
      return i;
    }
  }
  // If the pose was not found, return -1
 printf("Pose passed in getIndex is not contained within any wave cells.\n");
  return -1;
}

