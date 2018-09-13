#ifndef VISIONPOINTALT_H
#define VISIONPOINTALT_H

#include <inttypes.h>

/// @ingroup vision

/* This stores information on color "runs" (populated during classifier->constructRuns())
    lbIndex is filled in later by blob formation*/

struct VisionPointAlt {
  VisionPointAlt(uint16_t x, uint16_t y, unsigned char c) {
    xi = x;
    xf = x;
    dx = 1;
    yi = y;
    yf = y;
    dy = 1;
    color = c;
    nextVP = nullptr;
  }
  
  uint16_t xi, xf, dx, yi, yf, dy;
  unsigned char color;
  VisionPointAlt * nextVP;
  uint16_t lbIndex;
  bool isValid;
};


#endif
