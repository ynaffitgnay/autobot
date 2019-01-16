#ifndef _WAVEFRONT_PROPAGATION_H_
#define _WAVEFRONT_PROPAGATION_H_


// Wavefront cell construct holds data for a single cell in the map
#include <planning/structures/WaveCell.h>

// Map data construct
#include <planning/structures/Grid.h>
#include <math/Point.h>


  
/**
* Wavefront construct
*/
class WavefrontPropagation {
private:
  std::vector<WaveCell> waveCells;    /*!< Vector of wave cells that store wavefront data for each cell */
  int map_size_;                             /*!< Number of cells in the map (should be equivalent to mapCells size and waveCells size */
  int num_rows_;                             /*!< Number of cell rows in the map */
  int num_cols_;                             /*!< Number of cell columns in the map */
  bool has_map_;                             /*!< True if map data has been sent to waveCells */
  bool is_initialized_;                      /*!< True if class was initialized successfully */
  
     
    /**
   * @brief Finds and sets valid neighbors of all wavefront cells. Occupied cells are invalid neighbors.
   */
  void findNeighbors(void);
  
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
  bool setStartIndex(Pose2D& startPose);
  
  /**
   * @brief Propogates the wavefront and stores data in wavefront cells
   * @param startPose Pose of the robot at the start of the plan
   * @return True if the wavefront was propogated successfully
   */
  bool fill(Pose2D& startPose, bool swap);

    /**
   * @brief Generates an initial plan by ascending the gradient of values
   * @param plan Grid cells whose order value will be altered
   * @return True if all cells successfully ordered
   */
  bool traverse(std::vector<GridCell>& plan);
 
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

   /**
   * @brief Finds and returns the next closest valid and unplanned cell to the cell at the passed index.
   * @param index The index of the cell that the wavefront algorithm got stuck at.
   * @return The index of the closest valid and unplanned cell to the cell that was passed into the function.
   */
  int hop(int index);

  /**
   * @brief Adds the pose of the cell at the passed index to the plan accounting for orientation between the new cell and the previous cell in the plan.
   * @param lastI The index of the previous cell in the plan
   * @param i     The index of the cell to add to the plan
   * @param plan  The vector of poses that makes up the wavefront plan
   */
  void addPose(int lastI, int i, std::vector<GridCell>& plan, std::vector<GridCell>& orig_cells);

public:
  /**
   * @brief Constructor.
   */
  WavefrontPropagation();

  /**
   * @brief Destructor.
   */
  ~WavefrontPropagation();

  int start_index_;                          /*!< Index of the cell that the plan should start at (current position of the robot) */
  int end_index_;                            /*!< Index of the cell that the plan will end at (adjacent to start cell) */

  /**
   * @brief Initializes the wavefront.
   * @param map Map data
   * @param ot  Occupancy threshold for the map grid
   * @return True if initialization was successful
   */
  bool getCosts(Grid& map, Pose2D& startPose, bool swap, int lastVisitedIdx=-1);
  
  
  
}; // WavefrontPropagation

#endif // _WAVEFRONT_PROPAGATION_H_
