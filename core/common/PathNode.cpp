#include "PathNode.h"
#include <climits>

PathNode::PathNode() : PathNode(-1, -1) {
}

PathNode::PathNode(int row, int col) {
  r = row;
  c = col;
  idx = row * GRID_WIDTH + col;
  pred = -1;
  h = -1;
  g = INT_MAX;
  rhs = INT_MAX;
  consistent = false;
  visited = false;
  numVisits = 0;
  overlapped = false;
  reachable = true;
}

void PathNode::visit() {
  visited = true;
  ++numVisits;
}

//TODO: fix these to deal with keys
bool PathNode::operator==(const PathNode& other) const {
  return (std::min(g, rhs) == std::min(other.g, other.rhs));
}

bool PathNode::operator!=(const PathNode& other) const {
  return !(*this == other);
}

// larger key means smaller priority
bool PathNode::operator<(const PathNode& other) const {
  return (std::min(g, rhs) > std::min(other.g, other.rhs));
}

bool PathNode::operator<=(const PathNode& other) const {
  return (std::min(g, rhs) >= std::min(other.g, other.rhs));
}

// smaller key means larger priority
bool PathNode::operator>(const PathNode& other) const {
  return (std::min(g, rhs) < std::min(other.g, other.rhs));
}

bool PathNode::operator>=(const PathNode& other) const {
  return (std::min(g, rhs) <= std::min(other.g, other.rhs));
}

int PathNode::getIdx(int row, int col) {
  return (row * GRID_WIDTH + col);
}
