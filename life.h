/*
 * Game Of Life on Tilera, copyright (C) Gadi Oxman 2012
 */
#ifndef _LIFE_H
#define _LIFE_H

/*
 * Grid size in cells
 */
#if 1
#define GRID_COLS 256
#define GRID_ROWS 192
#define ROTATE_GRID 0
#else
// reversed grid
#define GRID_COLS 192
#define GRID_ROWS 256
#define ROTATE_GRID 1
#endif

// macro to round x up so it's divisable by y
#define ROUNDUP(x,y) ((((x) + (y) - 1) / y) * y)

// number of 32-bit cells in grid (rounding up GRID_COLS)
#define GRID_COLS32 (ROUNDUP(GRID_COLS,32)/32)

/*
 * L1 cache block size in 32 bit words
 *
 * 8 for SCC, 4 for Tilera
 */
#ifdef SCC_PLATFORM
#include "RCCE_lib.h"
#define L1_SIZE (RCCE_LINE_SIZE/4)
#else
#define L1_SIZE 4
#endif

/*
 * fifo header width. In the uncompressed version, this needs modification
 * to support cells modification detection for grids wider than 1024 cells.
 *
 * The round-up is for L1 block cache alignment
 */
#define FIFO_HEADER_WIDTH ROUNDUP(ROUNDUP(GRID_COLS32,32)/32,L1_SIZE)
//#define FIFO_HEADER_WIDTH 1

/*
 * Each line contains the header and the cells, 1 bit per cell.
 */
#define FIFO_LINE_WIDTH ROUNDUP(GRID_COLS32+FIFO_HEADER_WIDTH,L1_SIZE)
//#define FIFO_LINE_WIDTH (GRID_COLS32+FIFO_HEADER_WIDTH)

/*
 * FIFO size
 */
#ifdef SCC_PLATFORM
#define FIFO_SIZE (1920/FIFO_LINE_WIDTH*FIFO_LINE_WIDTH)
#else
#define FIFO_SIZE (2*(GRID_ROWS)*FIFO_LINE_WIDTH)
//#define FIFO_SIZE (16*FIFO_LINE_WIDTH)
#endif

extern char *input_file_name;
extern char *output_file_name;
extern int algorithm;
extern int num_steps;
extern int num_tiles;
#endif /* _LIFE_H */

