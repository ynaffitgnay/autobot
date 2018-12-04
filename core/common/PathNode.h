#ifndef PATHNODE_H
#define PATHNODE_H

#pragma once

#include <planning/PlanningConstants.h>
#include <common/GridCell.h>
#include <common/Serialization.h>
#include <common/DSLKey.h>
#include <schema/gen/PathNode_generated.h>

DECLARE_INTERNAL_SCHEMA(class PathNode {
  public:
    SCHEMA_METHODS(PathNode);
    PathNode(GridCell& cell);
    void visit();
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
    
    SCHEMA_FIELD(int idx);          // id of this node
    SCHEMA_FIELD(GridCell& cell);   // GridCell that this PathNode corresponds to
    SCHEMA_FIELD(int pred);         // id of this node's predecessor
    //SCHEMA_FIELD(int r);            // Y-coordinate (row of the grid cell)
    //SCHEMA_FIELD(int c);            // X coordinate (column of the grid cell)
    //SCHEMA_FIELD(int h);            // heuristic value
    SCHEMA_FIELD(int g);            // g-value
    SCHEMA_FIELD(int rhs);          // rhs-value
    SCHEMA_FIELD(DSLKey key);
    SCHEMA_FIELD(bool changed);  // is this node consistent?
    SCHEMA_FIELD(bool visited);     // Has this node been visited
    SCHEMA_FIELD(int numVisits);    // How many times has this node been visited?
    SCHEMA_FIELD(bool overlapped);  // does this overlap with the CCP?
    SCHEMA_FIELD(bool initialized); // Has this cell been initialized? TODO: remove?
    SCHEMA_FIELD(bool planned);   // Is this a free space?
    SCHEMA_FIELD(int pathorder);  
});

#endif
