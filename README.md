The complete coverage path planning code is organized into several different
blocks. The bulk of the code is contained in /core/planning, which is reproduced
in the zipped /planning directory.

# core/planning:
Majority of files are contained in Planning folder under "core/"
To run the project,
1. run the UT tool
2. select the behavior coverage.py for the robot

## PlanningModule.cpp
Contains the main logic for planning. Determines when to call D* Lite for
replanning and whether the planner should use D* Lite or A* to plan the
coverage path.

Planning information is passed between modules using the Planning memory block.

## GridGenerator.cpp
Creates the occupancy grid representation of the field.

## WavefrontGenerator.cpp
Assigns costs to each cell using a wavefront path transform.

## CoverageDSL.cpp, DStarLite.cpp
DStarLite contains the D* Lite algorithm for re-planning. CoverageDSL is a
derived class that creates a coverage path rather than the point-to-point
planning in DStarLite.

## ParticleFilter.cpp

### IntersectionDetector.cpp
Heuristics for markers on the field

### ObstacleDetector.cpp
Heuristics for markers on the field. Obstacles are only detected when the
python coverage behavior indicates that the Nao is facing toward the next
cell on the coverage path.

## coverage.py
Python behavior and controller

# Simulation
## WorldObject.h
WO_OBSTACLE_UNKNOWN objects have been added at the locations specified
in UTField.h to be placed around the field. The simulated scenarios
contain different configurations of these objects.

## init.py
The python interpreter was updated to also process each planning frame when it
runs. The initial behavior was changed to coverage.py to be run in simulation.

## CommandLineProcessor.cpp
This file contains the code to run the CCD* Lite and A* simulations. The
unknown obstacle configurations used for each simulation are also listed here.

## BasicGL.cpp, ObjectsGL.cpp
Code was added to be able to properly draw the field line configuration in the
robotics lab.

## GLDrawer.cpp
Various planning-related tools for drawing in the world window were added
including: drawing the planned path, drawing previous planned paths, drawing
the robot's believed path, drawing the simulated robot's true path, drawing
the known and unknown obstacles, and overlaying the current planning information
on the field.

## IsolatedBehaviorSimulation.cpp
The starting location of the player was changed to the starting cell for complete
coverage path planning. Constructors were added to choose the configuration
of unknown beacons that would be used in the simulated path as well as choosing
whether the created simulated player would use A* or CCD* Lite path planning.

## ObservationGenerator.cpp
The observation generator was updated to observe an obstacle in the next
gridcell on the path with 100% certainty. The generator was also modified to
use the planning block to inform the planning module that the path will need
to be changed, similar to the vision module's purpose on the real robot.

## SimulatedPlayer.cpp
The simulated player was updated to process a planning frame after observations
are generated.




  

  

  