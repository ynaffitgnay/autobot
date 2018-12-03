#ifndef PLANNING_CONSTANTS_H
#define PLANNING_CONSTANTS_H

#define FIELD_WIDTH  (3000)
#define FIELD_HEIGHT (2500)
#define CELL_HEIGHT  (200)
#define CELL_WIDTH   (200)
#define GRID_HEIGHT  (13)  // ~2480 mm / 200 mm
#define GRID_WIDTH   (15)  // ~2980 mm / 200 mm
#define GRID_SIZE    (GRID_WIDTH * GRID_HEIGHT)
#define PATH_SIZE    (4 * GRID_SIZE)


#define START_X  (1400)
#define START_Y  (-1060)


#define getGridRow(y) (int)((1250 - (y)) / CELL_HEIGHT)
#define getGridCol(x) (int)((1500 + (x)) / CELL_WIDTH)
#define getCellIdx(r,c) (int)((r) * GRID_WIDTH + (c))

#endif
