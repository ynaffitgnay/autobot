#include "DStarLite.h"

DStarLite::DStarLite(MemoryCache& cache, TextLogger*& tlogger)
  : cache_(cache), tlogger_(tlogger) {
}
  
void DStarLite::init(const Grid& wavefront) {
}

void DStarLite::runDSL() {
}

int DStarLite::calcKey(PathNode successor) {

  //TODO: don't return 0
  return 0;
}

void DStarLite::updateVertex(PathNode u) {
}

void DStarLite::computeShortestPath() {
}

