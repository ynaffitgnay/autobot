#ifndef _FIELD_H
#define _FIELD_H

#include "WorldObject.h"
#include <stdio.h>

//#ifdef interface_H
//cout << "Field_h";
//#endif


// setup constants for field size, landmark locations, etc

const float BALL_RADIUS = 31;

const float FIELD_Y = 2500;
const float FIELD_X = 3000;
const float GRASS_Y = 2800;
const float GRASS_X = 5000;

const float HALF_FIELD_Y = FIELD_Y/2.0;
const float HALF_FIELD_X = FIELD_X/2.0;
const float HALF_GRASS_Y = GRASS_Y/2.0;
const float HALF_GRASS_X = GRASS_X/2.0;

const float GOAL_Y = 900;
const float GOAL_POST_WIDTH = 70;
const float GOAL_WIDTH = GOAL_Y + GOAL_POST_WIDTH;
const float GOAL_X = 500;
const float HALF_GOAL_Y = GOAL_Y / 2.0;
const float PENALTY_Y = 1400;
const float PENALTY_X =  600;
const float CIRCLE_DIAMETER = 1205;
const float CIRCLE_RADIUS = CIRCLE_DIAMETER / 2.0;
const float LINE_WIDTH = 50;

const float GOAL_HEIGHT = 325;

const float FIELD_CENTER_X = 0;
const float FIELD_CENTER_Y = 0;

const float PENALTY_CROSS_X = HALF_FIELD_X - 1300;
const float PENALTY_MARK_SIZE = 106;

const float CIRCLE_HEX_LENGTH = 2.0*(CIRCLE_RADIUS*sinf(DEG_T_RAD*30.0));

// Some rectangles

const Rectangle FIELD =
Rectangle( Point2D( -FIELD_X / 2,  FIELD_Y / 2 ),
	   Point2D(  FIELD_X / 2, -FIELD_Y / 2 ) );

const Rectangle GRASS =
Rectangle( Point2D( -GRASS_X / 2,  GRASS_Y / 2 ),
	   Point2D(  GRASS_X / 2, -GRASS_Y / 2 ) );

// circle and cross points
const Point2D circleLocation = Point2D(0, 0);
const Point2D oppCrossLocation = Point2D(PENALTY_CROSS_X, 0);
const Point2D ownCrossLocation = Point2D(-PENALTY_CROSS_X, 0);


// Landmark locations
const Point2D landmarkLocation[] = {
  Point2D(-HALF_FIELD_X, 0),  // Semi-circle
  Point2D(-1530.0,1220.0),                    // WO_BEACON_BLUE_YELLOW
  Point2D(-1530.0,-1280.0),                   // WO_BEACON_YELLOW_BLUE
  Point2D(0.0, 1230.0),                       // WO_BEACON_BLUE_PINK
  Point2D(0.0, -1300.0),                      // WO_BEACON_PINK_BLUE
  Point2D(1500.0,1240.0),                     // WO_BEACON_PINK_YELLOW
  Point2D(1500.0,-1240.0),                    // WO_BEACON_YELLOW_PINK

  Point2D(300.0, 500.0),                      // WO_OBSTACLE_1
  Point2D(-600.0, -100.0),                    // WO_OBSTACLE_2

  Point2D( -FIELD_X / 2, 0),                  // WO_OWN_GOAL
  Point2D( FIELD_X / 2, 0 ),                  // WO_OPP_GOAL

  Point2D( -FIELD_X / 2, -GOAL_Y / 2),        // WO_OWN_LEFT_GOALPOST
  Point2D( FIELD_X / 2, GOAL_Y / 2 ),         // WO_OPP_LEFT_GOALPOST

  Point2D( -FIELD_X / 2, GOAL_Y / 2),         // WO_OWN_RIGHT_GOALPOST
  Point2D( FIELD_X / 2, -GOAL_Y / 2)          // WO_OPP_RIGHT_GOALPOST
};


// Set unknown obstacle locations for different replanning
const Point2D obstacleLocation[] = {
  Point2D(-1200.0, 1100.0),   // WO_OBSTACLE_UNKNOWN_1
  Point2D(-1200.0, 800.0),    // WO_OBSTACLE_UNKNOWN_2
  Point2D(-900.0, 500.0),     // WO_OBSTACLE_UNKNOWN_3
  Point2D(-300.0, 500.0),     // WO_OBSTACLE_UNKNOWN_4
  Point2D(900.0, 500.0),      // WO_OBSTACLE_UNKNOWN_5
  Point2D(1200.0, 500.0),     // WO_OBSTACLE_UNKNOWN_6
  Point2D(600.0, 200.0),      // WO_OBSTACLE_UNKNOWN_7
  Point2D(-1200.0, -100.0),   // WO_OBSTACLE_UNKNOWN_8
  Point2D(300.0, -100.0),     // WO_OBSTACLE_UNKNOWN_9
  Point2D(-300.0, -400.0),    // WO_OBSTACLE_UNKNOWN_10
  Point2D(600.0, -400.0),     // WO_OBSTACLE_UNKNOWN_11
  Point2D(-900.0, -700.0),    // WO_OBSTACLE_UNKNOWN_12
  Point2D(300.0, -700.0),     // WO_OBSTACLE_UNKNOWN_13
  Point2D(0.0, -1000.0),      // WO_OBSTACLE_UNKNOWN_14
  Point2D(300.0, -1000.0)    // WO_OBSTACLE_UNKNOWN_15
};


// Line intersection locations
const Point2D intersectionLocation[] = {
  // L
  Point2D( FIELD_X / 2, FIELD_Y / 2),                 // 0 WO_OPP_FIELD_LEFT_L
  Point2D( FIELD_X / 2, -FIELD_Y / 2),                //   WO_OPP_FIELD_RIGHT_L
  Point2D( 900.0, 680.0),                             // 2 WO_OPP_PEN_LEFT_L
  Point2D( 900.0, -690.0),                            //   WO_OPP_PEN_RIGHT_L
  Point2D( -FIELD_X / 2 - PENALTY_X, PENALTY_Y / 2),  // 4 WO_OWN_PEN_RIGHT_L
  Point2D( -FIELD_X / 2 - PENALTY_X, -PENALTY_Y / 2), //   WO_OWN_PEN_LEFT_L
  Point2D( -FIELD_X / 2, FIELD_Y / 2),                // 6 WO_OWN_FIELD_RIGHT_L
  Point2D( -FIELD_X / 2, -FIELD_Y / 2),               // 7 WO_OWN_FIELD_LEFT_L

  Point2D( -FIELD_X / 2, FIELD_Y / 2),                //   WO_OWN_FIELD_EDGE_TOP_L
  Point2D( FIELD_X / 2, FIELD_Y / 2),                 //   WO_OPP_FIELD_EDGE_TOP_L
  Point2D( -FIELD_X / 2, -FIELD_Y / 2),               //   WO_OWN_FIELD_EDGE_BOTTOM_L
  Point2D( FIELD_X / 2, -FIELD_Y / 2),                //   WO_OPP_FIELD_EDGE_BOTTOM_L
  
  
  Point2D(  HALF_FIELD_X + GOAL_X, -HALF_GOAL_Y),      // Back right of opp goal post
  Point2D(  HALF_FIELD_X + GOAL_X,  HALF_GOAL_Y),      // Back left of opp goal post

  Point2D( -HALF_FIELD_X - GOAL_X, -HALF_GOAL_Y),      // Back right of own goal post
  Point2D( -HALF_FIELD_X - GOAL_X,  HALF_GOAL_Y),      // Back left of own goal post

  // T
  Point2D( 1500.0, 700.0),                            // 8  WO_OPP_PEN_LEFT_T
  Point2D( 1500.0, -650.0),                           //    WO_OPP_PEN_RIGHT_T
  Point2D( 0, FIELD_Y / 2),                           // 10 WO_CENTER_TOP_T
  Point2D( 0, -FIELD_Y / 2),                          //    WO_CENTER_BOTTOM_T,
  Point2D( -1530.0, 650.0),                           // 12 WO_OWN_PEN_RIGHT_T
  Point2D( -1500.0, -700.0),                          // 13 WO_OWN_PEN_LEFT_T
  
  Point2D(  HALF_FIELD_X, -HALF_GOAL_Y),              // front right of opp goal
  Point2D(  HALF_FIELD_X,  HALF_GOAL_Y),              // front left of opp goal

  Point2D( -HALF_FIELD_X, -HALF_GOAL_Y),              // front right of own goal
  Point2D( -HALF_FIELD_X,  HALF_GOAL_Y)               // front left of own goal
};


// Line location
const Point2D lineLocationStarts[] = {

  // HORIZONTAL LINES
  intersectionLocation[WO_OPP_FIELD_LEFT_L  - INTERSECTION_OFFSET],    // WO_OPP_GOAL_LINE
  intersectionLocation[WO_OPP_PEN_LEFT_L    - INTERSECTION_OFFSET],    // WO_OPP_PENALTY
  intersectionLocation[WO_CENTER_TOP_T      - INTERSECTION_OFFSET],    // WO_CENTER_LINE
  intersectionLocation[WO_OWN_PEN_RIGHT_L   - INTERSECTION_OFFSET],    // WO_OWN_PENALTY
  intersectionLocation[WO_OWN_FIELD_RIGHT_L - INTERSECTION_OFFSET],    // WO_OWN_GOAL_LINE

  intersectionLocation[WO_OWN_FIELD_EDGE_TOP_L - INTERSECTION_OFFSET], // WO_OWN_FIELD_EDGE_TOP
  intersectionLocation[WO_OPP_FIELD_EDGE_TOP_L - INTERSECTION_OFFSET], // WO_OPP_FIELD_EDGE_TOP
  
  intersectionLocation[WO_OPP_BACK_RIGHT_GOAL_L - INTERSECTION_OFFSET],
  intersectionLocation[WO_OWN_BACK_RIGHT_GOAL_L - INTERSECTION_OFFSET],

  // VERTICAL LINES
  intersectionLocation[WO_OPP_FIELD_LEFT_L  - INTERSECTION_OFFSET],    // WO_TOP_SIDE_LINE
  intersectionLocation[WO_OPP_PEN_LEFT_L    - INTERSECTION_OFFSET],    // WO_PENALTY_TOP_OPP
  intersectionLocation[WO_OWN_PEN_RIGHT_L   - INTERSECTION_OFFSET],    // WO_PENALTY_TOP_OWN
  intersectionLocation[WO_OPP_PEN_RIGHT_L   - INTERSECTION_OFFSET],    // WO_PENALTY_BOTTOM_OPP
  intersectionLocation[WO_OWN_PEN_LEFT_L    - INTERSECTION_OFFSET],    // WO_PENALTY_BOTTOM_OWN
  intersectionLocation[WO_OPP_FIELD_RIGHT_L - INTERSECTION_OFFSET],    // WO_BOTTOM_SIDE_LINE

  // NEED TOP_FIELD_EDGE and BOTTOM_FIELD EDGE
  intersectionLocation[WO_OWN_FIELD_EDGE_TOP_L - INTERSECTION_OFFSET], // WO_TOP_FIELD_EDGE
  intersectionLocation[WO_OWN_FIELD_EDGE_BOTTOM_L - INTERSECTION_OFFSET], // WO_BOTTOM_FIELD_EDGE
  

  intersectionLocation[WO_OPP_BACK_LEFT_GOAL_L  - INTERSECTION_OFFSET], // WO_OPP_LEFT_GOALBAR
  intersectionLocation[WO_OPP_BACK_RIGHT_GOAL_L - INTERSECTION_OFFSET], // WO_OPP_RIGHT_GOALBAR
  intersectionLocation[WO_OWN_BACK_LEFT_GOAL_L  - INTERSECTION_OFFSET], // WO_OWN_LEFT_GOALBAR
  intersectionLocation[WO_OWN_BACK_RIGHT_GOAL_L - INTERSECTION_OFFSET]  // WO_OWN_RIGHT_GOALBAR
};

// Line location
const Point2D lineLocationEnds[] = {

  // HORIZONTAL LINES
  intersectionLocation[WO_OPP_FIELD_RIGHT_L - INTERSECTION_OFFSET],    // WO_OPP_GOAL_LINE
  intersectionLocation[WO_OPP_PEN_RIGHT_L   - INTERSECTION_OFFSET],    // WO_OPP_PENALTY
  intersectionLocation[WO_CENTER_BOTTOM_T   - INTERSECTION_OFFSET],    // WO_CENTER_LINE
  intersectionLocation[WO_OWN_PEN_LEFT_L    - INTERSECTION_OFFSET],    // WO_OWN_PENALTY
  intersectionLocation[WO_OWN_FIELD_LEFT_L  - INTERSECTION_OFFSET],    // WO_OWN_GOAL_LINE

  intersectionLocation[WO_OWN_FIELD_EDGE_BOTTOM_L - INTERSECTION_OFFSET], // WO_OWN_FIELD_EDGE_BOTTOM
  intersectionLocation[WO_OPP_FIELD_EDGE_BOTTOM_L - INTERSECTION_OFFSET], // WO_OPP_FIELD_EDGE_BOTTOM
  
  intersectionLocation[WO_OPP_BACK_LEFT_GOAL_L - INTERSECTION_OFFSET], // opp back goalbar
  intersectionLocation[WO_OWN_BACK_LEFT_GOAL_L - INTERSECTION_OFFSET], // own back goalbar

  // VERTICAL LINES
  intersectionLocation[WO_OWN_FIELD_RIGHT_L - INTERSECTION_OFFSET],    // WO_TOP_SIDE_LINE
  intersectionLocation[WO_OPP_PEN_LEFT_T    - INTERSECTION_OFFSET],    // WO_PENALTY_TOP_OPP
  intersectionLocation[WO_OWN_PEN_RIGHT_T   - INTERSECTION_OFFSET],    // WO_PENALTY_TOP_OWN
  intersectionLocation[WO_OPP_PEN_RIGHT_T   - INTERSECTION_OFFSET],    // WO_PENALTY_BOTTOM_OPP
  intersectionLocation[WO_OWN_PEN_LEFT_T    - INTERSECTION_OFFSET],    // WO_PENALTY_BOTTOM_OWN
  intersectionLocation[WO_OWN_FIELD_LEFT_L  - INTERSECTION_OFFSET],    // WO_BOTTOM_SIDE_LINE

  intersectionLocation[WO_OPP_FIELD_EDGE_TOP_L - INTERSECTION_OFFSET], // WO_TOP_FIELD_EDGE
  intersectionLocation[WO_OPP_FIELD_EDGE_BOTTOM_L - INTERSECTION_OFFSET], // WO_BOTTOM_FIELD_EDGE
  

  intersectionLocation[WO_OPP_FRONT_LEFT_GOAL_T  - INTERSECTION_OFFSET], // WO_OPP_LEFT_GOALBAR
  intersectionLocation[WO_OPP_FRONT_RIGHT_GOAL_T - INTERSECTION_OFFSET], // WO_OPP_RIGHT_GOALBAR
  intersectionLocation[WO_OWN_FRONT_LEFT_GOAL_T  - INTERSECTION_OFFSET], // WO_OWN_LEFT_GOALBAR
  intersectionLocation[WO_OWN_FRONT_RIGHT_GOAL_T - INTERSECTION_OFFSET]  // WO_OWN_RIGHT_GOALBAR
};


#endif
