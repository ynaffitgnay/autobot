#ifndef PLANNING_CONSTANTS_H
#define PLANNING_CONSTANTS_H

#define FIELD_HEIGHT (2500)
#define FIELD_WIDTH  (3000)
#define CELL_HEIGHT  (400)
#define CELL_WIDTH   (400)
#define GRID_HEIGHT  (6) //(13)  // ~2480 mm / 200 mm
#define GRID_WIDTH   (8) //(15)  // ~2980 mm / 200 mm
#define GRID_SIZE    (GRID_WIDTH * GRID_HEIGHT)
#define PATH_SIZE    (4 * GRID_SIZE)


#define START_X  (1400)
#define START_Y  (-1060)


#define getGridRow(y) (int)((1250 - (y)) / CELL_HEIGHT)
#define getGridCol(x) (int)((1500 + (x)) / CELL_WIDTH)
#define getCellIdx(r,c) (int)((r) * GRID_WIDTH + (c))
#define getColFromIdx(x) (int)(x % GRID_WIDTH)
#define getRowFromIdx(y) (int)((y - getColFromIdx(y)) / GRID_WIDTH)

#endif
