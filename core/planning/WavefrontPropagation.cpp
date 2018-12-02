#include "WavefrontPropagation.h"


WavefrontPropagation::WavefrontPropagation() :
  map_size_(0),
  num_rows_(0),
  num_cols_(0),
  start_index_(0),
  end_index_(0),
  has_map_(false),
  is_initialized_(false)
{
  
}

WavefrontPropagation::~WavefrontPropagation() {
  // Do nothing
}

bool WavefrontPropagation::getCosts(Grid& map, Pose2D& startPose) {
  // Set map variables
  map_size_ = map.cells.size();
  num_rows_ = std::round(float(map.height)/float(map.cell_height));
  num_cols_ = std::round(float(map.width)/float(map.cell_width));

  printf("Start pose: [%f, %f]\n",startPose.translation.x, startPose.translation.y);
  
  // Check for invalid parameters
  if(num_cols_ <= 0)
  {
    printf("Grid must have at least one column.\n");
    return false;
  }
  else if(num_rows_ <= 0)
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
  has_map_ = true;
  
  // Figure out which cells are neighbors
  findNeighbors();
  
  // Set init flag
  is_initialized_ = true;

  if (fill(startPose)) {
  	printf("Success!\n");
    for(int i = 0; i<waveCells.size(); i++) {
      map.cells[i].cost = waveCells[i].getValue();
      float globX = map.cells[i].center.translation.x;  // TODO: Needs a conversion from local x,y to global x,y not r,c
      float globY = map.cells[i].center.translation.y;
      float globTh = map.cells[i].center.rotation;
      Pose2D globCenter(globTh,globX,globY);
      map.cells[i].center = globCenter;
    }
  } else {
  	printf("Failure! :(\n");
    return false;
  }

  if (traverse(map.cells)) {
    printf("Successfully generated initial path order\n");
  } else {
    printf("Failed to generate initial path order\n");
    return false;
  }
  return true;
}

bool WavefrontPropagation::fill(Pose2D& startPose) {
  // Set start state if it is valid
  if(!setStartIndex(startPose)) {
    return false;
  }
  // Set start and end cell values
  waveCells[start_index_].setValue(WAVE_START);
  waveCells[end_index_].setValue(WAVE_END);
  
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
  int grid_cols = num_cols_;
  int grid_rows = num_rows_;
  int index;

  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  GridCell* cell;

  std::cout << "------------------Wave-Values-----------------------------------" << std::endl;
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

bool WavefrontPropagation::traverse(std::vector<GridCell> &orig_cells) {
  // Use debugPoses vector to print out map of plan order
  std::vector<GridCell> plan = orig_cells;
  std::vector<int> debugPoses;
  debugPoses.clear();
  
  // Determine how many cells are to be in the full coverage plan
  int numPoses = 0;     // number of poses we need to add to the plan
  for(int i = 0; i<waveCells.size(); i++)
  {
    // Cells to be planned do not include cells that the wavefront propogation could not reach, obstructions, or cells planned in a previous plan
    if(waveCells[i].getValue() != WAVE_INIT && waveCells[i].getValue() != WAVE_OBSTRUCTION && !waveCells[i].isVisited())
    {
      numPoses++;
    }
  }
  
  // Plan order of poses
  int index = start_index_;                    // index of the current cell being added to the plan
  addPose(start_index_-1, start_index_, plan, orig_cells);   // Add the start pose as the first pose in the plan
  debugPoses.push_back(start_index_);          // Make the start pose the first pose in debug_poses
  waveCells[index].visit();                  // visit the start cell since it has been planned
  waveCells[end_index_].visit();               // visit the end cell so it will not be planned (it will always be added at the end)
  int numPlanned = 2;                        // start and end cell
  while(numPlanned < numPoses)               // while we still have poses that need to be added to the plan
  {
    bool stuck = false;
    while(!stuck && numPlanned<=numPoses)  // if we get stuck, we exit the loop and use the hop function to find the closest unplanned cell
    {
      std::vector<WaveCell*> unvisited = waveCells[index].getUnvisitedNeighbors(); // vector of valid unvisited neighbors
      if(unvisited.size() > 0) // We can only move to a neighbor if it is valid and unvisited
      {
        int maxValue = unvisited[0]->getValue(); // value of the neighbor who's wavefront number is the highest
        int maxIndex = 0;                       // index of the neighbor who's wavefront number is the highest
        if(unvisited.size() > 1) // If there is more than one unvisited neighbor, find the one with the highest wavefront value
        {
          for(int i = 1; i<unvisited.size(); i++)
          {
            if(unvisited[i]->getValue() > maxValue)
            {
              maxValue = unvisited[i]->getValue();
              maxIndex = i;
            }
          }
        }
        int lastIndex = index;
        index = getIndex(unvisited[maxIndex]->getPosition()); // set the index to the neighbor we will move to
        // printf("Unvisted index: %d\n", index);
        addPose(lastIndex, index, plan, orig_cells);            // add the new cell to the plan
        debugPoses.push_back(index);
        numPlanned++;
      }
      else // if all valid neighbors have been visited, we are stuck
      {
        //printf("Stuck: no neighbors are unvisited.\n");
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
      addPose(lastIndex, index, plan, orig_cells); // add new cell to the plan
      debugPoses.push_back(index);
      numPlanned++;
    }
  }
  addPose(index, end_index_, plan, orig_cells); // add end cell to the plan
  debugPoses.push_back(end_index_);
  
  printf("Orig cell size: %d Debug pose size: %d", orig_cells.size(), debugPoses.size());

  std::vector<GridCell> ordered_plan;
  for (int i = 0; i < orig_cells.size(); i++){
    int order = 0;
    std::vector<int>::iterator it = std::find(debugPoses.begin(),debugPoses.end(),i);
    if(it == debugPoses.end())
      order = -1;
    else
      order = std::distance(debugPoses.begin(), it);
    orig_cells[i].order_index = order;
  }

  for (int i = 0; i < orig_cells.size(); i++){
    int debug_ind = debugPoses[i];
    ordered_plan.push_back(orig_cells[debug_ind]);
  }
  
  /**************************************************************************************************/
  /** This code block will print the order of the wavefront plan. Use only for debugging purposes. **/
  /**************************************************************************************************/
  int grid_cols = num_cols_;
  int grid_rows = num_rows_;

  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  GridCell cell;

  std::cout << "---------------------Debug-Poses-------------------------------" << std::endl;
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      index = row*grid_cols+col; // Finding the appropriate cell in vectorized form
      if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
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

  std::cout << "----------------Original-Cells-Altered--------------------------" << std::endl;
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      index = row*grid_cols+col; // Finding the appropriate cell in vectorized form
      if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
        std::cout<<color_red<< "|  |" <<default_col<<"|";
      else
      {
        int order = orig_cells[index].order_index;
        
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
  return true;
}

int WavefrontPropagation::hop(int index)
{
  // Find the closest unvisited cell
  bool found = false;        // true if we have found the closest unvisited cell
  std::vector<int> checked;  // vector of indices of cells that we have already checked to be unvisited
  checked.clear();
  checked.push_back(index);  // put the current cell on the list since it should hop back to itself
  std::vector<WaveCell*> neighbors; // vector of neighbors propogating outward from original cell
  neighbors.clear();
  neighbors.push_back(&waveCells[index]); // we start propogation from the cell we got stuck at
  while(!found && neighbors.size()!=0) // keep going until we find a valid hop, or we have checked all valid cells and therefore neighbors is empty
  {
    int numOld = neighbors.size(); // number of neighbors in the previous iteration
    int i = 0; // iteration of old neighbors
    while(i<numOld && !found) // iterate all old neighbors unless we find a valid cell to hop to
    {
      std::vector<WaveCell*> newNeighbors = neighbors[i]->getNeighbors(); // neighbors of our old neighbors (this is how we propogate outward)
      int j = 0; // iteration of new neighbors
      while(j<newNeighbors.size() && !found) // check all new neighbors unless one of them is found to be a valid hop
      {
  if(std::find(checked.begin(), checked.end(), getIndex(newNeighbors[j]->getPosition())) == checked.end()) // if the new neighbor has not already been checked
  {
    int r, c;
    getCoordinate(getIndex(newNeighbors[j]->getPosition()), r, c);
    neighbors.push_back(newNeighbors[j]); // move the new neighbor to the old neighbors list so that in the next iteration we check its neighbors
    checked.push_back(getIndex(newNeighbors[j]->getPosition())); // add this neighbor to the already checked list
    if(!newNeighbors[j]->isVisited())  // if it has not been visited, we found a valid hop
    {
      index = getIndex(newNeighbors[j]->getPosition());
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
    printf("We ran out of neighbors. Hop was called when all poses had already been planned\n");
  }
  return index;
}

void WavefrontPropagation::addPose(int lastI, int i, std::vector<GridCell>& plan, std::vector<GridCell>& orig_cells)
{
  // Get the center pose of the cell to be added to the plan
  Pose2D pose = waveCells[i].getPosition();
  
  // Determine which direction the robot will be coming so we know what orientation it should stop in
  if(i-lastI == 1) // the new cell is directly to the right of the old cell
  {
    pose.rotation = 0;
  }
  else if(i-lastI == -1) // the new cell is directly to left of the old cell
  {
    pose.rotation = 3.14;
  }
  else if(i-lastI < -1) // the new cell is far to the left or above the old cell
  {
    pose.rotation = 1.57;
  }
  else // the new cell is far to the right or below the old cell
  {
    pose.rotation = -1.57;
  }

  // add the pose to the plan
  plan.push_back(orig_cells[i]);
  
  // tell the cell it has been planned
  waveCells[i].visit();
}




bool WavefrontPropagation::setStartIndex(Pose2D& startPose) {
  // Find the cell that contains the start index
  int cellIndex = getIndex(startPose);
  printf("Cell Index: %d\n", cellIndex);
  
  // Make sure the cell is valid and not obstructed
  if (checkPose(startPose)) {
    start_index_ = cellIndex;
  }
  else {
    return false;
  }

  // Find a valid neighbor to place end index
  std::vector<WaveCell*> startNeighbors = waveCells[start_index_].getNeighbors();
  if(startNeighbors.size() > 0)
  {
    Pose2D endPose = startNeighbors[0]->getPosition();
    // printf("Num neighbors: %d End pose [%f, %f]\n",startNeighbors.size(), endPose.translation.x, endPose.translation.y);
    end_index_ = getIndex(endPose);
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
  if(cell_index >= 0 && cell_index < map_size_)
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
  if(!has_map_)
  {
    printf("Cannot get coordinate before mapdata is set.\n");
    return false;
  }

  if(index<0 || index >= map_size_)
  {
    printf("Index out of range. Cannot get coordinate\n");
    return false;
  }

  c = index % num_cols_;
  r = (index -c)/num_cols_;
  return true;
}

bool WavefrontPropagation::getIndex(int &index, int r, int c) {
  // Return the index corresponding to (r,c) if it is in the map
  if(!has_map_)
  {
    printf("Cannot get index before map data is set.\n");
    return false;
  }

  if(r < 0 || c < 0 || r >= num_rows_ || c >= num_cols_)
  {
    return false;
  }
  else
  {
    index = r*num_cols_ + c;
    return true;
  }
}

int WavefrontPropagation::getIndex(Pose2D pose) {
  // Return the index of the cell that contains pose
  // printf("Getting index of [%f, %f]\n",pose.translation.x, pose.translation.y);
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

