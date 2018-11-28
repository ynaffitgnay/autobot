#ifndef _WAVEFRONT_PROPAGATION_H_
#define _WAVEFRONT_PROPAGATION_H_


// Wavefront cell construct holds data for a single cell in the map
#include <planning/structures/WavefrontCell.h>

// Map data construct
#include <planning/structures/Grid.h>
#include <math/Point.h>


  
/**
* Wavefront construct
*/
class WavefrontPropagation {
private:
  std::vector<WavefrontCell> waveCells;    /*!< Vector of wave cells that store wavefront data for each cell */
  int mapSize;                             /*!< Number of cells in the map (should be equivalent to mapCells size and waveCells size */
  int numRows;                             /*!< Number of cell rows in the map */
  int numCols;                             /*!< Number of cell columns in the map */
  int startIndex;                          /*!< Index of the cell that the plan should start at (current position of the robot) */
  int endIndex;                            /*!< Index of the cell that the plan will end at (adjacent to start cell) */
  bool hasMap;                             /*!< True if map data has been sent to waveCells */
  bool isInitialized;                      /*!< True if class was initialized successfully */
  
    /**
   * @brief Adds the pose of the cell at the passed index to the plan accounting for orientation between the new cell and the previous cell in the plan.
   * @param lastI The index of the previous cell in the plan
   * @param i     The index of the cell to add to the plan
   * @param plan  The vector of poses that makes up the wavefront plan
   */
  void addPose(int lastI, int i, std::vector<Pose3D>& plan);
  
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
  void addNeighbor(std::vector<WavefrontCell*> &neighbors, int r, int c);
  
    /**
   * @brief Finds and returns the next closest valid and unplanned cell to the cell at the passed index.
   * @param index The index of the cell that the wavefront algorithm got stuck at.
   * @return The index of the closest valid and unplanned cell to the cell that was passed into the function.
   */
  int hop(int index);
  
    /**
   * @brief Sets the start index to the cell that startPose is contained in if it is a valid place to start.
   * @param startPose The pose that the robot at the start of the plan
   * @return True if the start index was set successfully
   */
  bool setStartIndex(Pose3D startPose);
  
public:
  /**
   * @brief Constructor.
   */
  Wavefront();

  /**
   * @brief Destructor.
   */
  ~Wavefront();

  /**
   * @brief Initializes the wavefront.
   * @param map Map data
   * @param ot  Occupancy threshold for the map grid
   * @return True if initialization was successful
   */
  bool getCosts(Grid& map, Point startPose);

  /**
   * @brief Checks if a pose is in a valid cell in the grid
   * @param pose The pose to check
   * @return True if the pose is in a valid cell in the grid
   */
  bool checkPose(Pose3D pose);
  
  /**
   * @brief Propogates the wavefront and stores data in wavefront cells
   * @param startPose Pose of the robot at the start of the plan
   * @return True if the wavefront was propogated successfully
   */
  bool fill(Pose3D startPose);
  
  /**
   * @brief Creates a full coverage plan based on the propogated wavefront.
   * @param plan Stores the wavefront plan
   */
  void getPlan(std::vector<Pose3D> &plan);
  
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
  int getIndex(Pose3D pose);
  
  /**
   * @brief Resets data of cells that contains each pose in the given vector
   * @param cells Vector of poses that are within cells whose data should be reset
   */
  void resetCells(std::vector<Pose3D> cells);

  /**
   * @brief Re-initializes the wavefront propogation
   */
  void reinit();
  
}; // WavefrontPropagation

#endif // _WAVEFRONT_PROPAGATION_H_
