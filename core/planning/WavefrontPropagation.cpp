#include "WavefrontPropagation.h"


Wavefront::Wavefront() :
  mapSize(0),
  numRows(0),
  numCols(0),
  startIndex(0),
  endIndex(0),
  hasMap(false),
  isInitialized(false)
{
  
}

Wavefront::~Wavefront()
{
  // Do nothing
}

bool Wavefront::getCosts(Grid& map, Pose2D& startPose)
{
  // Set map variables
  mapSize = map.cells.size();
  numRows = std::round(height/cell_height);
  numCols = std::round(width/cell_width);
  
  // Check for invalid parameters
  if(numCols <= 0)
  {
    printf("Grid must have at least one column.");
    return false;
  }
  else if(numRows <= 0)
  {
    printf("Grid must have at least one row.");
    return false;
  }

  // Initialize wave cells
  waveCells.clear();
  waveCells.resize(map.cells.size());
  for(int i = 0; i<waveCells.size(); i++)
  {
  	WaveCell wc(maps.cells[i].center, map.cells[i]);
    waveCells[i]=wc;
  }
  hasMap = true;
  
  // Figure out which cells are neighbors
  findNeighbors();
  
  // Set init flag
  isInitialized = true;

  if (fill(startPose)){
  	printf("Success!");
  	return true;
  } else {
  	printf("Failure! :(");
  	return false;
  }
}

bool Wavefront::fill(Pose2D startPose)
{
  // Set start state if it is valid
  if(!setStartIndex(startPose))
  {
    return false;
  }
  // Set start and end cell values
  waveCells[startIndex].setCell(WAVE_START);
  waveCells[endIndex].setCell(WAVE_END);
  
  // Fill remaining cells using wavefront propogation
  int fillCount = 2;      // Number of cells filled with wave numbers (not including obstructions)
  int waveNumber = 1;     // Number indicating the current wave
  int lastFillCount = 0;  // Number of cells filled with wave numbers in previous iteration
  while(fillCount != lastFillCount)
  {
    lastFillCount = fillCount;
    for(int i = 0; i<mapSize; i++)
    {
      if(waveCells[i].getCell() == waveNumber-1)
      {
        fillCount += waveCells[i].setNeighbors(waveNumber);
      }
    }
    waveNumber++;
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
  floor_coverage::SquareCell* cell;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      index = row*grid_cols+col; // Finding the appropriate cell in vectorized form
      if (waveCells[index].getCell() == WAVE_OBSTRUCTION) // If the cell is an obstacle
        std::cout<<color_red<< "| |" <<default_col<<"|";
      else if(waveCells[index].isVisited())
      {
        if(waveCells[index].getCell()<10 && waveCells[index].getCell()>=0)
          std::cout << "| "<< color_red << waveCells[index].getCell() << default_col << "|"; 
        else
          std::cout << "|"<< color_red << waveCells[index].getCell() << default_col << "|"; 
      }
      else
      {
        if(waveCells[index].getCell()<10 && waveCells[index].getCell()>=0)
          std::cout << "| "<< waveCells[index].getCell() <<"|"; 
        else
          std::cout << "|"<< waveCells[index].getCell() <<"|"; 
      }
    }
    std::cout << std::endl;
  }
  std::cout << "----------------------------------------------------------------" << std::endl;
  
  return true;
}



bool Wavefront::setStartIndex(Pose2D startPose)
{
  // Find the cell that contains the start index
  int cellIndex = getIndex(startPose);
  printf("Cell Index: %d", cellIndex);
  
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
    endIndex = startNeighbors[0]->getPosition();
    return true;
  }
  else
  {
    printf_THROTTLE(5, "Start pose invalid: has no valid neighbors");
    return false;
  }
}

bool Wavefront::checkPose(Pose2D pose) {
  int cell_index = getIndex(pose);
  if(cell_index >= 0 && cell_index < mapSize)
  {
    if(WAVE_OBSTRUCTION == waveCells[cell_index].getCell())
    {
      printf_THROTTLE(5, "Start pose invalid: cell is occupied");
      return false;
    }
    return true;
  }
  else
  {
    printf_THROTTLE(5, "Start pose invalid: not within map bounds.");
    return false;
  }
}

void Wavefront::findNeighbors()
{
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

void Wavefront::addNeighbor(std::vector<WaveCell*> &neighbors, int r, int c)
{
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

bool Wavefront::getCoordinate(int index, int &r, int &c)
{
  //return the coordinate corresponding to index if it is withing the map
  if(!hasMap)
  {
    printf("Cannot get coordinate before mapdata is set.");
    return false;
  }

  if(index<0 || index >= mapSize)
  {
    printf("Index out of range. Cannot get coordinate");
    return false;
  }

  c = index % numCols;
  r = (index -c)/numCols;
}

bool Wavefront::getIndex(int &index, int r, int c)
{
  // Return the index corresponding to (r,c) if it is in the map
  if(!hasMap)
  {
    printf("Cannot get index before map data is set.");
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

int Wavefront::getIndex(Pose2D pose)
{
  // Return the index of the cell that contains pose
  for(int i = 0; i<waveCells.size(); i++)
  {
    if(waveCells[i].contains(pose)) {
      return i;
    }
  }
  // If the pose was not found, return -1
  printf_THROTTLE(5, "Pose passed in getIndex is not contained within any wave cells.");
  return -1;
}

