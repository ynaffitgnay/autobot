#ifndef P_N_CMP_H
#define P_N_CMP_H

#pragma once

#include <planning/structures/PathNode.h>

struct PNCmp {
  // Organize heap such that the value with the largest priority has the smallest key
  inline bool operator()(const PathNode* left, const PathNode* right) { return *left > *right; }
};

#endif
