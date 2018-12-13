# PlanningModule.cpp
Majority of files are contained in Planning folder under "core/"
To run the project,
1. run tool
2. run coverage.py

## GridGenerator.cpp
Makes the grid

## WavefrontGenerator.cpp
Creates the coverage path plan for execution

## CoverageDSL.cpp, DStarLite.cpp
DStarLite contains the D* Lite algorithm for re-planning

## ParticleFilter.cpp

### IntersectionDetector.cpp
Heuristics for markers on the field

### ObstacleDetector.cpp
Heuristics for markers on the field

## coverage.py
Python behavior and controller