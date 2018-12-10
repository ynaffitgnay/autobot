#ifndef PATHNODE_H
#define PATHNODE_H

#pragma once

#include <planning/PlanningConstants.h>
#include <common/GridCell.h>
#include <common/DSLKey.h>

class PathNode {
  public:
    PathNode(GridCell& cell);
    void setOccupied();
    bool operator==(const PathNode& other) const;
    bool operator!=(const PathNode& other) const;
    bool operator<(const PathNode& other) const;
    bool operator<=(const PathNode& other) const;
    bool operator>(const PathNode& other) const;
    bool operator>=(const PathNode& other) const;
    const int getValue();
    
    static int getIdx(int row, int col);
    static bool getGridCoordinate(int index, int &row, int &col);
    
    int idx;          // id of this node
    GridCell& cell;   // GridCell that this PathNode corresponds to
    int pred;         // id of this node's predecessor
    //int r;            // Y-coordinate (row of the grid cell)
    //int c;            // X coordinate (column of the grid cell)
    //int h;            // heuristic value
    int g;            // g-value
    int rhs;          // rhs-value
    DSLKey key;
    bool changed;     // Has this node changed?
    //bool visited;     // Has this node been visited
    int numVisits;    // How many times has this node been visited?
    bool overlapped;  // does this overlap with the CCP?
    bool initialized; // Has this cell been initialized? TODO: remove?
    bool planned;   // Is this a free space?
    int pathorder;  
};

#endif
