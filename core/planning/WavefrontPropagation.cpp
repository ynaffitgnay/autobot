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

bool Wavefront::getCosts(CellGrid map, int ot)
{
  // Set map variables
  mapSize = map.grid_cells.size();
  numCols = map.grid_cols;
  
  // Check for invalid parameters
  if(numCols <= 0)
  {
    ROS_ERROR("Grid must have at least one column.");
    return false;
  }
  else if(mapSize % numCols != 0)
  {
    ROS_ERROR("Grid vector size must be divisible by the number of columns.");
    return false;
  }

  // Calculate number of rows
  numRows = mapSize/numCols;
  
  // Initialize wave cells
  waveCells.clear();
  waveCells.resize(map.grid_cells.size());
  for(int i = 0; i<waveCells.size(); i++)
  {
    waveCells[i].initialize(i, map.grid_cells[i], ot);
  }
  hasMap = true;
  
  // Figure out which cells are neighbors
  findNeighbors();
  
  // Set init flag
  isInitialized = true;
}

bool Wavefront::fill(Pose3D startPose)
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

void Wavefront::getPlan(std::vector<Pose3D> &plan)
{
  // Use debugPoses vector to print out map of plan order
  std::vector<int> debugPoses;
  debugPoses.clear();
  
  // Determine how many cells are to be in the full coverage plan
  int numPoses = 0;     // number of poses we need to add to the plan
  for(int i = 0; i<waveCells.size(); i++)
  {
    // Cells to be planned do not include cells that the wavefront propogation could not reach, obstructions, or cells planned in a previous plan
    if(waveCells[i].getCell() != WAVE_INIT && waveCells[i].getCell() != WAVE_OBSTRUCTION && !waveCells[i].isVisited())
    {
      numPoses++;
    }
  }
  
  // Plan order of poses
  int index = startIndex;                    // index of the current cell being added to the plan
  addPose(startIndex-1, startIndex, plan);   // Add the start pose as the first pose in the plan
  debugPoses.push_back(startIndex);          // Make the start pose the first pose in debug_poses
  waveCells[index].visit();                  // visit the start cell since it has been planned
  waveCells[endIndex].visit();               // visit the end cell so it will not be planned (it will always be added at the end)
  int numPlanned = 2;                        // start and end cell
  while(numPlanned < numPoses)               // while we still have poses that need to be added to the plan
  {
    bool stuck = false;
    while(!stuck && numPlanned<=numPoses)  // if we get stuck, we exit the loop and use the hop function to find the closest unplanned cell
    {
      std::vector<WavefrontCell*> unvisited = waveCells[index].getUnvisitedNeighbors(); // vector of valid unvisited neighbors
      if(unvisited.size() > 0) // We can only move to a neighbor if it is valid and unvisited
      {
        int maxValue = unvisited[0]->getCell(); // value of the neighbor who's wavefront number is the highest
        int maxIndex = 0;                       // index of the neighbor who's wavefront number is the highest
        if(unvisited.size() > 1) // If there is more than one unvisited neighbor, find the one with the highest wavefront value
        {
          for(int i = 1; i<unvisited.size(); i++)
          {
            if(unvisited[i]->getCell() > maxValue)
            {
              maxValue = unvisited[i]->getCell();
              maxIndex = i;
            }
          }
        }
        int lastIndex = index;
        index = unvisited[maxIndex]->getPosition(); // set the index to the neighbor we will move to
        addPose(lastIndex, index, plan);            // add the new cell to the plan
        debugPoses.push_back(index);
        numPlanned++;
      }
      else // if all valid neighbors have been visited, we are stuck
      {
        //ROS_INFO("Stuck: no neighbors are unvisited.");
        stuck = true;
      }
    }
    
    if(numPlanned<numPoses) // if we exited the previous loop because we were stuck, not because we were done planning
    {
      // When we get stuck, find the closest unvisited cell and go there
      int lastIndex = index;
      index = hop(index); // hop to the closed unvisited cell
      if(index == lastIndex) // if hop was unsuccessful, we should must not have any more cells to plan
        numPlanned = numPoses;
      addPose(lastIndex, index, plan); // add new cell to the plan
      debugPoses.push_back(index);
      numPlanned++;
    }
  }
  addPose(index, endIndex, plan); // add end cell to the plan
  debugPoses.push_back(endIndex);
  
  /**************************************************************************************************/
  /** This code block will print the order of the wavefront plan. Use only for debugging purposes. **/
  /**************************************************************************************************/
  int grid_cols = numCols;
  int grid_rows = numRows;

  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  floor_coverage::SquareCell cell;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      index = row*grid_cols+col; // Finding the appropriate cell in vectorized form
      if (waveCells[index].getCell() == WAVE_OBSTRUCTION) // If the cell is an obstacle
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      else
      {
        int order = 0;
        std::vector<int>::iterator it = std::find(debugPoses.begin(),debugPoses.end(),index);
        if(it == debugPoses.end())
          order = -1;
        else
          order = std::distance(debugPoses.begin(), it);
        
        if(order<0)
          std::cout<<color_red<< "|  |" <<default_col<<"|";
        else if(order<10)
          std::cout << "|  "<< order <<"|"; 
  else if(order<100)
          std::cout << "| "<< order <<"|"; 
        else
          std::cout << "|"<< order <<"|"; 
      }
    }
    std::cout << std::endl;
  }
  std::cout << "----------------------------------------------------------------" << std::endl;
}

int Wavefront::hop(int index)
{
  // Find the closest unvisited cell
  bool found = false;        // true if we have found the closest unvisited cell
  std::vector<int> checked;  // vector of indices of cells that we have already checked to be unvisited
  checked.clear();
  checked.push_back(index);  // put the current cell on the list since it should hop back to itself
  std::vector<WavefrontCell*> neighbors; // vector of neighbors propogating outward from original cell
  neighbors.clear();
  neighbors.push_back(&waveCells[index]); // we start propogation from the cell we got stuck at
  while(!found && neighbors.size()!=0) // keep going until we find a valid hop, or we have checked all valid cells and therefore neighbors is empty
  {
    int numOld = neighbors.size(); // number of neighbors in the previous iteration
    int i = 0; // iteration of old neighbors
    while(i<numOld && !found) // iterate all old neighbors unless we find a valid cell to hop to
    {
      std::vector<WavefrontCell*> newNeighbors = neighbors[i]->getNeighbors(); // neighbors of our old neighbors (this is how we propogate outward)
      int j = 0; // iteration of new neighbors
      while(j<newNeighbors.size() && !found) // check all new neighbors unless one of them is found to be a valid hop
      {
  if(std::find(checked.begin(), checked.end(), newNeighbors[j]->getPosition()) == checked.end()) // if the new neighbor has not already been checked
  {
    int r, c;
    getCoordinate(newNeighbors[j]->getPosition(), r, c);
    neighbors.push_back(newNeighbors[j]); // move the new neighbor to the old neighbors list so that in the next iteration we check its neighbors
    checked.push_back(newNeighbors[j]->getPosition()); // add this neighbor to the already checked list
    if(!newNeighbors[j]->isVisited())  // if it has not been visited, we found a valid hop
    {
      index = newNeighbors[j]->getPosition();
      found = true;
    }
  }
  j++;
      }
      i++;
    }
    if(numOld == 1) // delete the neighbors who ran through the algorithm in the last iteration so we don't keep rechecking neighbors
      neighbors.erase(neighbors.begin());
    else
      neighbors.erase(neighbors.begin(), neighbors.begin()+numOld);
  }
  if(!found && neighbors.size()==0) // the algorithm failed (this means we have already planned all poses and didn't need to hop in the first place)
  {
    ROS_ERROR("We ran out of neighbors. Hop was called when all poses had already been planned");
  }
  return index;
}

bool Wavefront::setStartIndex(Pose3D startPose)
{
  // Find the cell that contains the start index
  int cellIndex = getIndex(startPose);
  ROS_INFO("Cell Index: %d", cellIndex);
  
  // Make sure the cell is valid and not obstructed
  if (checkPose(startPose)) {
    startIndex = cellIndex;
  }
  else {
    return false;
  }

  // Find a valid neighbor to place end index
  std::vector<WavefrontCell*> startNeighbors = waveCells[startIndex].getNeighbors();
  if(startNeighbors.size() > 0)
  {
    endIndex = startNeighbors[0]->getPosition();
    waveCells[endIndex].reset(); // reset the end index. This is for replanning.. we need a place to end even if it has been visited.
    return true;
  }
  else
  {
    ROS_ERROR_THROTTLE(5, "Start pose invalid: has no valid neighbors");
    return false;
  }
}

bool Wavefront::checkPose(Pose3D pose) {
  int cell_index = getIndex(pose);
  if(cell_index >= 0 && cell_index < mapSize)
  {
    if(WAVE_OBSTRUCTION == waveCells[cell_index].getCell())
    {
      ROS_ERROR_THROTTLE(5, "Start pose invalid: cell is occupied");
      return false;
    }
    return true;
  }
  else
  {
    ROS_ERROR_THROTTLE(5, "Start pose invalid: not within map bounds.");
    return false;
  }
}

void Wavefront::addPose(int lastI, int i, std::vector<Pose3D>& plan)
{
  // Get the center pose of the cell to be added to the plan
  Pose3D pose = waveCells[i].getPose();
  
  // // Determine which direction the robot will be coming so we know what orientation it should stop in
  // tf::Quaternion quat;
  // if(i-lastI == 1) // the new cell is directly to the right of the old cell
  // {
  //   quat.setRPY(0,0,0);
  // }
  // else if(i-lastI == -1) // the new cell is directly to left of the old cell
  // {
  //   quat.setRPY(0,0,3.14);
  // }
  // else if(i-lastI < -1) // the new cell is far to the left or above the old cell
  // {
  //   quat.setRPY(0,0,1.57);
  // }
  // else // the new cell is far to the right or below the old cell
  // {
  //   quat.setRPY(0,0,-1.57);
  // }
  // tf::quaternionTFToMsg(quat,pose.pose.orientation);
  
  // add the pose to the plan
  plan.push_back(pose);
  
  // tell the cell it has been planned
  waveCells[i].visit();
}

void Wavefront::findNeighbors()
{
  // Set up neighbors vectors for all cells
  std::vector<WavefrontCell*> neighbors; // vector of valid neighbors to be passed to cell
  for(int i = 0; i<waveCells.size(); i++)
  {
    if(WAVE_OBSTRUCTION != waveCells[i].getCell()) // cell does not need neighbors if it is occupied (and therefore inaccessible)
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

void Wavefront::addNeighbor(std::vector<WavefrontCell*> &neighbors, int r, int c)
{
  // add a neighbor to the neighbors array if (r,c) is valid neighbor
  int index = 0;
  if(getIndex(index, r, c)) // false if (r,c) is not within map
  {
    if(WAVE_OBSTRUCTION != waveCells[index].getCell()) // do not add obstructed neighbors
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
    ROS_ERROR("Cannot get coordinate before mapdata is set.");
    return false;
  }

  if(index<0 || index >= mapSize)
  {
    ROS_ERROR("Index out of range. Cannot get coordinate");
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
    ROS_ERROR("Cannot get index before map data is set.");
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

int Wavefront::getIndex(Pose3D pose)
{
  // Return the index of the cell that contains pose
  for(int i = 0; i<waveCells.size(); i++)
  {
    if(waveCells[i].contains(pose)) {
      return i;
    }
  }
  // If the pose was not found, return -1
  ROS_ERROR_THROTTLE(5, "Pose passed in getIndex is not contained within any wave cells.");
  return -1;
}

void Wavefront::resetCells(std::vector<Pose3D> cells)
{
  // Call reset on all cells in the cells vector
  for(int i = 0; i<cells.size(); i++)
  {
    int index = getIndex(cells[i]);
    if(index >= 0 && index < mapSize) // do not attempt to access a cell that does not exist
    {
      waveCells[index].reset(); 
    }
  }
}

void Wavefront::reinit()
{
  // reinitialize wavefront so we can propogate a new wave. Note this does not change visited value, just the wave value
  for(int i = 0; i<waveCells.size(); i++)
  {
    waveCells[i].setCell(WAVE_INIT);
  }
}

