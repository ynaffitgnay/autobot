#include "PathNode.h"
#include <climits>

PathNode::PathNode(GridCell& cell) : cell(cell) {
  idx = cell.r * GRID_WIDTH + cell.c;
  pred = -1;
  g = INT_MAX;
  rhs = INT_MAX;
  changed = false;
  visited = false;
  numVisits = 0;
  overlapped = false;
  initialized = false;
}


void PathNode::visit() {
  visited = true;
  ++numVisits;
}

void PathNode::setOccupied() {
  cell.occupied = true;
  g = INT_MAX;
  rhs = INT_MAX;
}

bool PathNode::operator==(const PathNode& other) const {
  return (key == other.key);
}

bool PathNode::operator!=(const PathNode& other) const {
  return !(*this == other);
}

bool PathNode::operator<(const PathNode& other) const {
  return (key < other.key);
}

bool PathNode::operator<=(const PathNode& other) const {
  return (*this < other || *this == other);
}

bool PathNode::operator>(const PathNode& other) const {
  return !(*this <= other);
}

bool PathNode::operator>=(const PathNode& other) const {
  return !(*this < other);
}

int PathNode::getIdx(int row, int col) {
  return (row * GRID_WIDTH + col);
}

bool PathNode::getGridCoordinate(int index, int &row, int &col) {
  if (index >= GRID_HEIGHT * GRID_WIDTH || index < 0) {
    return false;
  }

  col = index % GRID_WIDTH;
  row = (index - col) / GRID_HEIGHT;
  return true;
}
