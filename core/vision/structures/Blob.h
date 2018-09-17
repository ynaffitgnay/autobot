#ifndef BLOB_H
#define BLOB_H

#include <vision/VisionConstants.h>
#include <vector>
#include <inttypes.h>

/// @ingroup vision
struct Blob {
  unsigned char color;
  uint16_t xi, xf, dx, yi, yf, dy;
  uint16_t lpCount;
  std::vector<uint32_t> lpIndex;
  float diffStart;
  float diffEnd;
  float doubleDiff;
  uint16_t widthStart; // Width at yi
  uint16_t widthEnd; // Width at yf
  uint16_t avgX;    // Summation ((xi + xf)/2)/total pixels
  uint16_t avgY;    // Summation ((yi + yf)/2)/total pixels
  uint16_t total;
  float avgWidth;   // Summation (xf_row - xi_row)/total rows
  float correctPixelRatio;
  bool invalid;

  // GOAL DETECTION
  int edgeSize;
  int edgeStrength;

  Blob() : lpIndex(MAX_BLOB_VISIONPOINTS, 0) { }
  Blob(unsigned char c, uint16_t xi, uint16_t xf, uint16_t dx, uint16_t yi,
    uint16_t yf, uint16_t dy, uint16_t widthStart, uint16_t widthEnd, uint16_t avgX,
       uint16_t avgY, uint16_t total, float pixelRatio):
    color(c), xi(xi), xf(xf), dx(dx), yi(yi), yf(yf), dy(dy), widthStart(widthStart),
    widthEnd(widthEnd), avgX(avgX), avgY(avgY), total(total), correctPixelRatio(pixelRatio),
    lpIndex(MAX_BLOB_VISIONPOINTS, 0), lpCount(0) { }
};

/// @ingroup vision
bool sortBlobAreaPredicate(Blob* left, Blob* right);

#endif
