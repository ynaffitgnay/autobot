#ifndef _PATHNODE_H_
#define _PATHNODE_H_

#pragma once

#include <common/Serialization.h>
#include <schema/gen/PathNode_generated.h>

DECLARE_INTERNAL_SCHEMA(struct PathNode {
  SCHEMA_METHODS(PathNode);
  SCHEMA_FIELD(float x);         // X coordinate
  SCHEMA_FIELD(float y);         // Y coordinate
  SCHEMA_FIELD(int h);           // heuristic value
  SCHEMA_FIELD(int g);           // g-value
  SCHEMA_FIELD(int rhs);         // rhs-value
  SCHEMA_FIELD(bool consistent); // is this node consistent?
});

#endif
