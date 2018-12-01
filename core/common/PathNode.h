#ifndef PATHNODE_H
#define PATHNODE_H

#pragma once

#include <planning/PlanningConstants.h>
#include <common/Serialization.h>
#include <common/DSLKey.h>
#include <schema/gen/PathNode_generated.h>

DECLARE_INTERNAL_SCHEMA(class PathNode {
  public:
    SCHEMA_METHODS(PathNode);
    PathNode();
    PathNode(int row, int col);
    PathNode(int row, int col, int cost);
    void visit();
    bool operator==(const PathNode& other) const;
    bool operator!=(const PathNode& other) const;
    bool operator<(const PathNode& other) const;
    bool operator<=(const PathNode& other) const;
    bool operator>(const PathNode& other) const;
    bool operator>=(const PathNode& other) const;
    bool operator==(const DSLKey& other) const;
    bool operator!=(const DSLKey& other) const;
    bool operator<(const DSLKey& other) const;
    bool operator<=(const DSLKey& other) const;
    bool operator>(const DSLKey& other) const;
    bool operator>=(const DSLKey& other) const;

    static int getIdx(int row, int col);
    
    SCHEMA_FIELD(int idx);          // id of this node
    SCHEMA_FIELD(int pred);         // id of this node's predecessor
    SCHEMA_FIELD(int r);            // Y-coordinate (row of the grid cell)
    SCHEMA_FIELD(int c);            // X coordinate (column of the grid cell)
    SCHEMA_FIELD(int h);            // heuristic value
    SCHEMA_FIELD(int g);            // g-value
    SCHEMA_FIELD(int rhs);          // rhs-value
    //SCHEMA_FIELD(int k_1);          // 1st component of key for priority queue
    //SCHEMA_FIELD(int k_2);          // 2nd componentof key for priority queue
    SCHEMA_FIELD(DSLKey key);
    SCHEMA_FIELD(bool consistent);  // is this node consistent?
    SCHEMA_FIELD(bool visited);     // Has this node been visited
    SCHEMA_FIELD(int numVisits);    // How many times has this node been visited?
    SCHEMA_FIELD(bool overlapped);  // does this overlap with the CCP?
    SCHEMA_FIELD(bool initialized); // Has this cell been initialized? TODO: remove?
    SCHEMA_FIELD(bool reachable);   // Is this a free space?
});

#endif
