#ifndef OBSERVATION_GENERATOR_H
#define OBSERVATION_GENERATOR_H

#include <memory/MemoryCache.h>
#include <memory/WorldObjectBlock.h>
#include <math/Geometry.h>
#include <common/Random.h>
#include <common/ImageParams.h>
#include <localization/LocalizationConfig.h>


class ObservationGenerator {
  public:
    ObservationGenerator();
    ~ObservationGenerator();
    ObservationGenerator(const ObservationGenerator& og);
    void setObjectBlocks(WorldObjectBlock* gtObjects, WorldObjectBlock* obsObjects);
    void setObjectBlocks(WorldObjectBlock* gtObjects, std::vector<WorldObjectBlock*> obsObjects);
    void setInfoBlocks(FrameInfoBlock* frameInfo, JointBlock* joints);
    void setPlanningBlocks(PlanningBlock* planning, std::vector<WorldObjectType>* obstacles);
    void setModelBlocks(OpponentBlock* opponentMem);
    void setPlayer(int player, int team);
    void setImageParams(const ImageParams& iparams);
    void generateAllObservations();
    void generateGroundTruthObservations();
  private:
    void generateBallObservations();
    void generateLineObservations();
    void generateObstacleObservations();
    void generateOpponentObservations();
    void generateCenterCircleObservations();
    void generateGoalObservations();
    void generatePenaltyCrossObservations();
    void generateBeaconObservations();
    void fillObservationObjects();
    void initializeBelief();
    ImageParams iparams_;
    WorldObjectBlock *gt_object_, *obs_object_;
    OpponentBlock* opponent_mem_;
    PlanningBlock* planning_;
    FrameInfoBlock* frame_info_;
    JointBlock* joint_;
    int player_, team_;
    bool initialized_ = false;
    std::vector<WorldObjectBlock*> obs_objects_;
    std::vector<WorldObjectType>* obstacles_;
    LocalizationConfig lconfig_;

    std::vector<WorldObjectType> def_obstacles = {
      // WO_OBSTACLE_UNKNOWN_1,
      // WO_OBSTACLE_UNKNOWN_2,
      WO_OBSTACLE_UNKNOWN_3,
      WO_OBSTACLE_UNKNOWN_4,
      WO_OBSTACLE_UNKNOWN_5, 
      WO_OBSTACLE_UNKNOWN_6, 
      // WO_OBSTACLE_UNKNOWN_7, 
      // WO_OBSTACLE_UNKNOWN_8, 
      // WO_OBSTACLE_UNKNOWN_9, 
      // WO_OBSTACLE_UNKNOWN_10,
      // WO_OBSTACLE_UNKNOWN_11,
      WO_OBSTACLE_UNKNOWN_12,
      // WO_OBSTACLE_UNKNOWN_13,
      // WO_OBSTACLE_UNKNOWN_14,
      WO_OBSTACLE_UNKNOWN_15,
    };

};

#endif
