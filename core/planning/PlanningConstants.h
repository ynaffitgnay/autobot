#ifndef PLANNING_CONSTANTS_H
#define PLANNING_CONSTANTS_H

#define CELL_HEIGHT (200)
#define CELL_WIDTH  (200)
#define GRID_HEIGHT (13)  // ~2480 mm / 200 mm
#define GRID_WIDTH  (15)  // ~2980 mm / 200 mm

#define getGridRow(y) (int)((1240 - (y)) / CELL_HEIGHT)
#define getGridCol(x) (int)((1500 + (x)) / CELL_WIDTH)

#endif
