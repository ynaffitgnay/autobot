#ifndef PATHNODE_H
#define PATHNODE_H

#pragma once

#include <planning/PlanningConstants.h>
#include <common/Serialization.h>
#include <schema/gen/PathNode_generated.h>

DECLARE_INTERNAL_SCHEMA(class PathNode {
  public:
    SCHEMA_METHODS(PathNode);
    PathNode();
    PathNode(int x, int y);
    void visit();
    bool operator==(const PathNode& other) const;
    bool operator!=(const PathNode& other) const;
    bool operator<(const PathNode& other) const;
    bool operator<=(const PathNode& other) const;
    bool operator>(const PathNode& other) const;
    bool operator>=(const PathNode& other) const;
    static int getIdx(int x, int y);
    SCHEMA_FIELD(int idx);          // id of this node
    SCHEMA_FIELD(int pred);         // id of this node's predecessor
    SCHEMA_FIELD(int x);            // X coordinate
    SCHEMA_FIELD(int y);            // Y coordinate
    SCHEMA_FIELD(int h);            // heuristic value
    SCHEMA_FIELD(int g);            // g-value
    SCHEMA_FIELD(int rhs);          // rhs-value
    SCHEMA_FIELD(bool consistent);  // is this node consistent?
    SCHEMA_FIELD(bool visited);     // Has this node been visited
    SCHEMA_FIELD(int numVisits);    // How many times has this node been visited?
    SCHEMA_FIELD(bool overlapped);  // does this overlap with the CCP?
    SCHEMA_FIELD(bool reachable);   // Is this a free space?
});

#endif
