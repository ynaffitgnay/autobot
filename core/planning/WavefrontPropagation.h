#ifndef _WAVEFRONT_PROPAGATION_H_
#define _WAVEFRONT_PROPAGATION_H_


// Wavefront cell construct holds data for a single cell in the map
#include <planning/WaveCell.h>

// Map data construct
#include <planning/structures/Grid.h>
#include <math/Point.h>


  
/**
* Wavefront construct
*/
class WavefrontPropagation {
private:
  std::vector<WaveCell> waveCells;    /*!< Vector of wave cells that store wavefront data for each cell */
  int mapSize;                             /*!< Number of cells in the map (should be equivalent to mapCells size and waveCells size */
  int numRows;                             /*!< Number of cell rows in the map */
  int numCols;                             /*!< Number of cell columns in the map */
  int startIndex;                          /*!< Index of the cell that the plan should start at (current position of the robot) */
  int endIndex;                            /*!< Index of the cell that the plan will end at (adjacent to start cell) */
  bool hasMap;                             /*!< True if map data has been sent to waveCells */
  bool isInitialized;                      /*!< True if class was initialized successfully */
  
     
    /**
   * @brief Finds and sets valid neighbors of all wavefront cells. Occupied cells are invalid neighbors.
   */
  void findNeighbors();
  
    /**
   * @brief Adds neighbor at position (r,c) to the neighbors vector if it is valid.
   * @param neighbors Vector of neighbors
   * @param r         Row of proposed neighbor
   * @param c         Column of proposed neighbor
   */
  void addNeighbor(std::vector<WaveCell*> &neighbors, int r, int c);

    /**
   * @brief Sets the start index to the cell that startPose is contained in if it is a valid place to start.
   * @param startPose The pose that the robot at the start of the plan
   * @return True if the start index was set successfully
   */
  bool setStartIndex(Pose2D startPose);
  
public:
  /**
   * @brief Constructor.
   */
  WavefrontPropagation();

  /**
   * @brief Destructor.
   */
  ~WavefrontPropagation();

  /**
   * @brief Initializes the wavefront.
   * @param map Map data
   * @param ot  Occupancy threshold for the map grid
   * @return True if initialization was successful
   */
  bool getCosts(Grid& map, Pose2D& startPose);
  
  /**
   * @brief Propogates the wavefront and stores data in wavefront cells
   * @param startPose Pose of the robot at the start of the plan
   * @return True if the wavefront was propogated successfully
   */
  bool fill(Pose2D startPose);
 
  /**
   * @brief Checks if a pose is in a valid cell in the grid
   * @param pose The pose to check
   * @return True if the pose is in a valid cell in the grid
   */
  bool checkPose(Pose2D pose);
  
  /**
   * @brief Transforms index into (r,c) getCoordinate if the index is valid
   * @param index Index of the cell
   * @param r     Row number corresponding to the given index
   * @param c     Column number corresponding to the given index
   * @return      True if the index was valid
   */
  bool getCoordinate(int index, int &r, int &c);
  
  /**
   * @brief Transforms (r,c) getCoordinate into a cell index if the coordinate is valid
   * @param index Index of the cell at (r,c)
   * @param r     Row number of the cell
   * @param c     Column number of the cell
   * @return      True if the (r,c) coordinate was valid
   */
  bool getIndex(int &index, int r, int c);
  
  /**
   * @brief Determines which cell contains the given pose
   * @param pose The pose we want the cell index of
   * @return Index of the cell containing the given pose
   */
  int getIndex(Pose2D pose);
  
  
}; // WavefrontPropagation

#endif // _WAVEFRONT_PROPAGATION_H_
