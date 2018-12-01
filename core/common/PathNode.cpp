#include "PathNode.h"
#include <climits>

PathNode::PathNode() : PathNode(-1, -1) { }

PathNode::PathNode(int row, int col) : PathNode(row, col, -1) { }

PathNode::PathNode(int row, int col, int cost) {
  r = row;
  c = col;
  idx = row * GRID_WIDTH + col;
  pred = -1;
  h = cost;
  g = INT_MAX;
  rhs = INT_MAX;
  consistent = false;
  visited = false;
  numVisits = 0;
  overlapped = false;
  reachable = true;
  initialized = false;
}

void PathNode::visit() {
  visited = true;
  ++numVisits;
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

bool PathNode::operator==(const DSLKey& other) const {
  return (key == other);
}

bool PathNode::operator!=(const DSLKey& other) const {
  return !(*this == other);
}

bool PathNode::operator<(const DSLKey& other) const {
  return (key < other);
}

bool PathNode::operator<=(const DSLKey& other) const {
  return (*this < other || *this == other);
}

bool PathNode::operator>(const DSLKey& other) const {
  return !(*this <= other);
}

bool PathNode::operator>=(const DSLKey& other) const {
  return !(*this < other);
}


//From before adding key schema
//bool PathNode::operator==(const PathNode& other) const {
//  //return (std::min(g, rhs) == std::min(other.g, other.rhs));
//  return (k_1 == other.k_1 && k_2 == other.k_2);
//}
//
//bool PathNode::operator!=(const PathNode& other) const {
//  return !(*this == other);
//}
//
//bool PathNode::operator<(const PathNode& other) const {
//  if (k_1 < other.k_1) return true;
//  if (k_1 > other.k_1) return false;
//  // k_1 == other.k_1
//  return (k_2 < other.k_2);
//}
//
//bool PathNode::operator<=(const PathNode& other) const {
//  //return (std::min(g, rhs) >= std::min(other.g, other.rhs));
//  return (*this < other || *this == other);
//}
//
//bool PathNode::operator>(const PathNode& other) const {
//  return !(*this <= other);
//}
//
//bool PathNode::operator>=(const PathNode& other) const {
//  return !(*this < other);
//}

// THESE FUNCTIONS WERE FOR WHEN HEAP WAS USING STANDARD < OPERATOR
//// larger key means smaller priority
//bool PathNode::operator<(const PathNode& other) const {
//  //return (std::min(g, rhs) > std::min(other.g, other.rhs));
//  if (k_1 > other.k_1) return true;
//  if (k_1 < other.k_1) return false;
//  //if (k_1 == other.k_1) {
//  // k_1 == other.k_1
//  return (k_2 > other.k_2);
//}
//
//bool PathNode::operator<=(const PathNode& other) const {
//  //return (std::min(g, rhs) >= std::min(other.g, other.rhs));
//  return (*this < other || *this == other);
//}
//
//// smaller key means larger priority
//bool PathNode::operator>(const PathNode& other) const {
//  //return (std::min(g, rhs) < std::min(other.g, other.rhs));
//  //if (k_1 < other.k_1) return true;
//  //if (k_1 > other.k_1) return false;
//  //return (k_2 < other.k_2);
//  return !(*this <= other);
//}
//
//bool PathNode::operator>=(const PathNode& other) const {
//  //return (std::min(g, rhs) <= std::min(other.g, other.rhs));
//  return !(*this < other);
//}

int PathNode::getIdx(int row, int col) {
  return (row * GRID_WIDTH + col);
}
