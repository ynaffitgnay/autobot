#include "PathNode.h"
#include <climits>

PathNode::PathNode(GridCell& cell) : cell(cell) {
  idx = cell.r * GRID_WIDTH + cell.c;
  g = INT_MAX;
  rhs = INT_MAX;
  changed = false;
  numVisits = 0;
  planned = false;
  pathorder = -1;
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

const int PathNode::getValue() {
  return g;
}

int PathNode::getIdx(int row, int col) {
  return (row * GRID_WIDTH + col);
}

bool PathNode::getGridCoordinate(int index, int &row, int &col) {
  if (index >= GRID_HEIGHT * GRID_WIDTH || index < 0) {
    return false;
  }

  col = index % GRID_WIDTH;
  row = (index - col) / GRID_WIDTH;
  return true;
}
