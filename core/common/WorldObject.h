#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

#include <math/Geometry.h>
#include <common/Enum.h>
#include <common/Serialization.h>
#include <schema/gen/WorldObject_generated.h>

/**
 * \enum The types of world objects that exist.
 */
ENUM(WorldObjectType,   // Types of objects  // 0
  WO_BALL,  // 1

  //@{
  /// Self and Teammates
  WO_TEAM1,  // 2
  WO_TEAM2,  // 3
  WO_TEAM3,  // 4
  WO_TEAM4,  // 5
  WO_TEAM5,  // 6
  //@}

  // Opponents?
  WO_OPPONENT1,  // 7
  WO_OPPONENT2,  // 8
  WO_OPPONENT3,  // 9
  WO_OPPONENT4,  // 10
  WO_OPPONENT5,  // 11

  // Coaches
  WO_TEAM_COACH,  // 12
  WO_OPPONENT_COACH,  // 13
  WO_TEAM_LISTENER,  // 14

  /// center cicle   // LANDMARKS START HERE
  WO_CENTER_CIRCLE,  // 15
  
  WO_BEACON_BLUE_YELLOW,  // 16
  WO_BEACON_YELLOW_BLUE,  // 17
  WO_BEACON_BLUE_PINK,  // 18
  WO_BEACON_PINK_BLUE,  // 19
  WO_BEACON_PINK_YELLOW,  // 20
  WO_BEACON_YELLOW_PINK,  // 21

  WO_OBSTACLE_1, // 22
  WO_OBSTACLE_2, // 23
     
  // 2 Landmark goals
  WO_OWN_GOAL,  // 24
  WO_OPP_GOAL,  // 25

  WO_OWN_LEFT_GOALPOST,  // 26
  WO_OPP_LEFT_GOALPOST,  // 27

  WO_OWN_RIGHT_GOALPOST,  // 28
  WO_OPP_RIGHT_GOALPOST,  // 29  // AND END HERE

  // Unknown landmarks
  WO_OBSTACLE_UNKNOWN_1, // 30
  WO_OBSTACLE_UNKNOWN_2, // 31
  WO_OBSTACLE_UNKNOWN_3, // 32
  WO_OBSTACLE_UNKNOWN_4, // 33
     
  // Unknown goals and goal posts
  WO_UNKNOWN_GOAL,  // 28  --  CHANGED
  WO_UNKNOWN_LEFT_GOALPOST,  // 29  --  CHANGED
  WO_UNKNOWN_RIGHT_GOALPOST,  // 30  --  CHANGED
  WO_UNKNOWN_GOALPOST,  // 31  --  CHANGED

  // Field Corners
  WO_UNKNOWN_L_1,  // 32  --  CHANGED
  WO_UNKNOWN_L_2,  // 33  --  CHANGED
  WO_UNKNOWN_T_1,  // 34  --  CHANGED
  WO_UNKNOWN_T_2,  // 35  --  CHANGED

  // L intersections (yellow to blue, top to bottom)
  WO_OPP_FIELD_LEFT_L,  // 36  --  CHANGED
  WO_OPP_FIELD_RIGHT_L,  // 37  --  CHANGED
  WO_OPP_PEN_LEFT_L,  // 38  --  CHANGED
  WO_OPP_PEN_RIGHT_L,  // 39  --  CHANGED
  WO_OWN_PEN_RIGHT_L,  // 40  --  CHANGED
  WO_OWN_PEN_LEFT_L,  // 41  --  CHANGED
  WO_OWN_FIELD_RIGHT_L,  // 42  --  CHANGED
  WO_OWN_FIELD_LEFT_L,  // 43  --  CHANGED

  WO_OWN_FIELD_EDGE_TOP_L,  // 44  --  CHANGED
  WO_OPP_FIELD_EDGE_TOP_L,  // 45  --  CHANGED
  WO_OWN_FIELD_EDGE_BOTTOM_L,  // 46  --  CHANGED
  WO_OPP_FIELD_EDGE_BOTTOM_L,  // 47  --  CHANGED

  WO_OPP_BACK_RIGHT_GOAL_L,  // 48  --  CHANGED
  WO_OPP_BACK_LEFT_GOAL_L,  // 49  --  CHANGED
  
  WO_OWN_BACK_RIGHT_GOAL_L,  // 50  --  CHANGED
  WO_OWN_BACK_LEFT_GOAL_L,  // 51  --  CHANGED

  // T intersections (yellow to blue, top to bottom)
  WO_OPP_PEN_LEFT_T,  // 52  --  CHANGED
  WO_OPP_PEN_RIGHT_T,  // 53  --  CHANGED
  WO_CENTER_TOP_T,  // 54  --  CHANGED
  WO_CENTER_BOTTOM_T,  // 55  --  CHANGED
  WO_OWN_PEN_RIGHT_T,  // 56  --  CHANGED
  WO_OWN_PEN_LEFT_T,  // 57  --  CHANGED

  WO_OPP_FRONT_RIGHT_GOAL_T,  // 58  --  CHANGED
  WO_OPP_FRONT_LEFT_GOAL_T,  // 59  --  CHANGED

  WO_OWN_FRONT_RIGHT_GOAL_T,  // 60  --  CHANGED
  WO_OWN_FRONT_LEFT_GOAL_T,  // 61  --  CHANGED

  WO_UNKNOWN_FIELD_LINE_1,  // 62  --  CHANGED
  WO_UNKNOWN_FIELD_LINE_2,  // 63  --  CHANGED
  WO_UNKNOWN_FIELD_LINE_3,  // 64  --  CHANGED
  WO_UNKNOWN_FIELD_LINE_4,  // 65  --  CHANGED


  // Horizontal Lines (right to left)
  WO_OPP_GOAL_LINE,   // 66  // THIS IS WHERE LINES START            --  CHANGED
  WO_OPP_PENALTY,  // 67  --  CHANGED
  WO_CENTER_LINE,  // 68  --  CHANGED
  WO_OWN_PENALTY,  // 69  --  CHANGED
  WO_OWN_GOAL_LINE,  // 70  --  CHANGED

  WO_OWN_FIELD_EDGE,  // 71  --  CHANGED
  WO_OPP_FIELD_EDGE,  // 72  --  CHANGED
  
  WO_OPP_BOTTOM_GOALBAR,  // 73  --  CHANGED
  WO_OWN_BOTTOM_GOALBAR,  // 74  --  CHANGED

  // Vertical Lines (top to bottom)
  WO_TOP_SIDE_LINE,  // 75      --  CHANGED
  WO_PENALTY_TOP_OPP,  // 76     --  CHANGED
  WO_PENALTY_TOP_OWN,  // 77  --  CHANGED
  WO_PENALTY_BOTTOM_OPP,  // 78   --  CHANGED
  WO_PENALTY_BOTTOM_OWN,  // 79         --  CHANGED
  WO_BOTTOM_SIDE_LINE,  // 80     --  CHANGED
  
  WO_TOP_FIELD_EDGE,  // 81  --  CHANGED
  WO_BOTTOM_FIELD_EDGE,  // 82  --  CHANGED
  
  WO_OPP_LEFT_GOALBAR,  // 83  --  CHANGED
  WO_OPP_RIGHT_GOALBAR,  // 84  --  CHANGED
  WO_OWN_LEFT_GOALBAR,  // 85  --  CHANGED
  WO_OWN_RIGHT_GOALBAR,  // 86  // last line  --  CHANGED

  // penalty crosses
  WO_UNKNOWN_PENALTY_CROSS,  // 87  --  CHANGED
  WO_OWN_PENALTY_CROSS,  // 88  --  CHANGED
  WO_OPP_PENALTY_CROSS,  // 89  --  CHANGED

  // cluster of robots
  WO_ROBOT_CLUSTER,  // 90  --  CHANGED

  NUM_WORLD_OBJS,  // 94

  WO_INVALID  // 92  --  CHANGED

);


const int WO_OPPONENT_FIRST=WO_OPPONENT1;
const int WO_OPPONENT_LAST=WO_OPPONENT5;

const int WO_TEAM_FIRST=WO_TEAM1;
const int WO_TEAM_FIELD_FIRST=WO_TEAM2;
const int WO_TEAM_LAST=WO_TEAM5;
const int NUM_PLAYERS = WO_TEAM_LAST - WO_TEAM_FIRST + 1;

const int WO_PLAYERS_FIRST=WO_TEAM_FIRST;
const int WO_PLAYERS_LAST=WO_OPPONENT_LAST;
const int WO_ROBOTS_LAST=WO_OPPONENT_COACH;

// some constants to easily see which class each type is
const int WO_GOAL_FIRST = WO_OWN_GOAL;
const int WO_GOAL_LAST = WO_UNKNOWN_GOALPOST;
const int LANDMARK_OFFSET = WO_CENTER_CIRCLE;
const int NUM_LANDMARKS = WO_OPP_RIGHT_GOALPOST - LANDMARK_OFFSET + 1;
const int NUM_UNKNOWN_OBSTACLES = 4;
const int INTERSECTION_OFFSET =  WO_OPP_FIELD_LEFT_L;
const int NUM_INTERSECTIONS = WO_OWN_FRONT_LEFT_GOAL_T - INTERSECTION_OFFSET + 1;
const int LINE_OFFSET =  WO_OPP_GOAL_LINE;
const int NUM_LINES = WO_OWN_RIGHT_GOALBAR - LINE_OFFSET + 1;

const int L_INT_OFFSET = WO_OPP_FIELD_LEFT_L;
const int T_INT_OFFSET = WO_OPP_PEN_LEFT_T;

const int NUM_L_INTS = 8;
const int NUM_T_INTS = 6;

const int CROSS_OFFSET = WO_OWN_PENALTY_CROSS;
const int NUM_CROSSES = 2;

const int NUM_UNKNOWN_L = 2;
const int NUM_UNKNOWN_T = 2;
const int NUM_UNKNOWN_LINES = 4;
const int NUM_UNKNOWN_POSTS = 3;

#define WO_PROPERTY(name) \
  inline bool name() const { return name(type); } \
  inline static bool name(int type) { return name(static_cast<WorldObjectType>(type)); } \
  inline static bool name(WorldObjectType type)

/**
 * A WorldObject is used to refer to any object in the game. This
 * encompasses everything from robots, the ball, the goals, etc.
 * Everything interactable on the field is a world object.
 */
DECLARE_INTERNAL_SCHEMA(class WorldObject {
  public:
    SCHEMA_METHODS(WorldObject);
    WorldObject();
    WorldObject(WorldObjectType type);
    WorldObject(int type);
    ~WorldObject();

    // quick functions to determine which class of object this is
    WO_PROPERTY(isUnknown) {
      return isUnknownPost(type) || isUnknownGoalCenter(type) || isUnknownGoal(type) || isUnknownIntersection(type) || isUnknownLine(type) || isUnknownPenaltyCross(type) || isUnknownObstacle(type);
    }

    WO_PROPERTY(isLine) {
      return isUnknownLine(type) || isKnownLine(type);
    }

    WO_PROPERTY(isKnownLine) {
      return (type >= LINE_OFFSET && type < (LINE_OFFSET + NUM_LINES));
    }

    WO_PROPERTY(isLandmark) {
      return (type >= LANDMARK_OFFSET && type < (LANDMARK_OFFSET + NUM_LANDMARKS));
    }

    WO_PROPERTY(isUnknownObstacle) {
      return (type >= WO_OBSTACLE_UNKNOWN_1 && type <= WO_OBSTACLE_UNKNOWN_4);
    }

    WO_PROPERTY(isGoal) {
      return (type >= WO_OWN_GOAL && type <= WO_UNKNOWN_GOALPOST);
    }

    WO_PROPERTY(isGoalCenter) {
      return (type == WO_OPP_GOAL || type == WO_OWN_GOAL || type == WO_UNKNOWN_GOAL);
    }

    WO_PROPERTY(isGoalPost) {
      return (isGoal(type) && !isGoalCenter(type));
    }

    WO_PROPERTY(isUnknownPost) {
      return (type >= WO_UNKNOWN_LEFT_GOALPOST && type <= WO_UNKNOWN_GOALPOST);
    }

    WO_PROPERTY(isUnknownGoal) {
      return (type >= WO_UNKNOWN_GOAL && type <= WO_UNKNOWN_GOALPOST);
    }

    WO_PROPERTY(isUnknownGoalCenter) {
      return type == WO_UNKNOWN_GOAL;
    }

    WO_PROPERTY(isOwnGoal) {
      return (type == WO_OWN_GOAL || type == WO_OWN_RIGHT_GOALPOST || type == WO_OWN_LEFT_GOALPOST);
    }

    WO_PROPERTY(isOppGoal) {
      return (type == WO_OPP_GOAL || type == WO_OPP_RIGHT_GOALPOST || type == WO_OPP_LEFT_GOALPOST);
    }

    WO_PROPERTY(isBall) {
      return (type == WO_BALL);
    }

    WO_PROPERTY(isIntersection) {
      return isT(type) || isL(type);
    }

    WO_PROPERTY(isT) {
      return (type >= WO_OPP_PEN_LEFT_T && type <= WO_OWN_FRONT_LEFT_GOAL_T);
    }

    WO_PROPERTY(isL) {
      return (type >= WO_OPP_FIELD_LEFT_L && type <= WO_OWN_BACK_LEFT_GOAL_L);
    }

    WO_PROPERTY(isUnknownIntersection) {
      return (type >= WO_UNKNOWN_L_1 && type <= WO_UNKNOWN_T_2);
    }

    WO_PROPERTY(isUnknownT) {
      return (type >= WO_UNKNOWN_T_1 && type <= WO_UNKNOWN_T_2);
    }

    WO_PROPERTY(isUnknownL) {
      return (type >= WO_UNKNOWN_L_1 && type <= WO_UNKNOWN_L_2);
    }

    WO_PROPERTY(isUnknownLine) {
      return (type >= WO_UNKNOWN_FIELD_LINE_1 && type <= WO_UNKNOWN_FIELD_LINE_4);
    }

    WO_PROPERTY(isTeammate) {
      return (type >= WO_TEAM_FIRST && type <= WO_TEAM_LAST);
    }

    WO_PROPERTY(isOpponent) {
      return (type >= WO_OPPONENT_FIRST && type <= WO_OPPONENT_LAST);
    }

    WO_PROPERTY(isRobot) {
      return isTeammate(type) || isOpponent(type);
    }

    WO_PROPERTY(isCenterCircle) {
      return type == WO_CENTER_CIRCLE;
    }

    WO_PROPERTY(isUnknownPenaltyCross) {
      return type == WO_UNKNOWN_PENALTY_CROSS;
    }

    WO_PROPERTY(isKnownPenaltyCross) {
      return (type == WO_OPP_PENALTY_CROSS || type == WO_OWN_PENALTY_CROSS);
    }

    WO_PROPERTY(isPenaltyCross) {
      return isUnknownPenaltyCross(type) || isKnownPenaltyCross(type);
    }

    WO_PROPERTY(isUnique) {
      return isCenterCircle(type) || isKnownLine(type) || isKnownPenaltyCross(type);  // Center circle and known lines
    }

    WO_PROPERTY(isPenaltySideline) {
      return (type >= WO_PENALTY_TOP_OPP && type <= WO_PENALTY_BOTTOM_OWN);
    }

    WO_PROPERTY(isAmbiguous) {
      return (isUnknownGoal(type) || isUnknownIntersection(type) || isUnknownLine(type) || isUnknownPenaltyCross(type));
    }

    WO_PROPERTY(isHorizontalLine) {
      return type >= WO_OPP_GOAL_LINE && type <= WO_OWN_BOTTOM_GOALBAR;
    }

    WO_PROPERTY(isVerticalLine) {
      return type >= WO_TOP_SIDE_LINE && type <= WO_OWN_RIGHT_GOALBAR;
    }

    WO_PROPERTY(isBeacon) {
      return type >= WO_BEACON_BLUE_YELLOW && type <= WO_BEACON_YELLOW_PINK;
    }

    WO_PROPERTY(isObstacle) {
      return type >= WO_OBSTACLE_1 && type <= WO_OBSTACLE_2;
    }

    WO_PROPERTY(isEdgeIntersection) {
      return type >= WO_OWN_FIELD_EDGE_TOP_L && type <= WO_OPP_FIELD_EDGE_BOTTOM_L;
    }
    
    WO_PROPERTY(isEdgeLine) {
      if(type == WO_OWN_FIELD_EDGE) return true;
      if(type == WO_OPP_FIELD_EDGE) return true;
      if(type == WO_TOP_FIELD_EDGE) return true;
      if(type == WO_BOTTOM_FIELD_EDGE) return true;
      return false;
    }

    WO_PROPERTY(isEdgeObject) {
      return isEdgeIntersection(type) || isEdgeLine(type);
    }

    void reset();

    SCHEMA_FIELD(bool seen);
    SCHEMA_FIELD(int frameLastSeen);
    SCHEMA_FIELD(WorldObjectType type);
    SCHEMA_FIELD(WorldObjectType utype);


    // These are relative to the robot and come directly from vision
    SCHEMA_FIELD(float visionDistance); ///< Distance from the object to the robot, as estimated directly by vision.
    SCHEMA_FIELD(float visionElevation); ///< not used
    SCHEMA_FIELD(float visionBearing); ///< The relative angle between this object and the robot estimated directly by vision.
    SCHEMA_FIELD(float visionConfidence); ///< The confidence of the visionDistance and visionBearing estimates.

    // These are relative to the robot but come from the filtered localization
    SCHEMA_FIELD(float distance); ///< Relative distance to the robot.
    SCHEMA_FIELD(float elevation); ///< How high off the ground this object is relative to the robot.
    SCHEMA_FIELD(float bearing); ///< The relative angle between this object and the robot.

    SCHEMA_FIELD(Point2D relVel); ///< The relative velocity between the robot and this object.
    
    // These are the global position after localization (including confidence)
    // or permanent locations for landmarks and unmovable objects
    SCHEMA_FIELD(Point2D loc); ///< The object's location in global field coordinates.
    SCHEMA_FIELD(Point2D endLoc);
    SCHEMA_FIELD(float height); ///< The hight of the object from the field floor.

    // for lines
    // for unknown lines this is the relative x,y (x in front, y to left) of the end points of the observed line segment
    // for known lines this is the location on the field
    SCHEMA_FIELD(LineSegment lineLoc);

    // for seen lines
    SCHEMA_FIELD(Point2D visionPt1); ///< For lines, the first seen point.
    SCHEMA_FIELD(Point2D visionPt2); ///< For lines, the second seen point.
    SCHEMA_FIELD(LineSegment visionLine); ///< For lines, the line segment.

    // heights of beacons/goals/etc in the world
    SCHEMA_FIELD(float lowerHeight); ///< The height of the lower part of this object.
    SCHEMA_FIELD(float upperHeight); ///< The height of the upper part of this object.

    SCHEMA_FIELD(float orientation);
    SCHEMA_FIELD(Point2D sd);
    SCHEMA_FIELD(float sdOrientation);
    SCHEMA_FIELD(Point2D absVel);

    SCHEMA_FIELD(Point2D relPos);
    SCHEMA_FIELD(float relOrientation);

    // Vision
    SCHEMA_FIELD(int imageCenterX); ///< The object's center x in the frame image.
    SCHEMA_FIELD(int imageCenterY); ///< The object's center y in the frame image.
    SCHEMA_FIELD(float radius); ///< The radius of this object in the frame image.
    SCHEMA_FIELD(int fieldLineIndex);
    SCHEMA_FIELD(int ballBlobIndex);
    SCHEMA_FIELD(bool ballMoved); ///< This is only used by the keeper during penalty kicks.

    SCHEMA_FIELD(bool fromTopCamera); ///< True if observed from top cam, false if from bottom

    SCHEMA_FIELD(bool localized);

    // Obstacle
    SCHEMA_FIELD(float width);
    SCHEMA_FIELD(float length);

    // Obstacle
    SCHEMA_FIELD(float cwidth);
    SCHEMA_FIELD(float cheight);
});

typedef std::vector<WorldObject> ObjectSet;

#endif
