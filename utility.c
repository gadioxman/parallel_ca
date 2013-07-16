/*
 * Game Of Life on Tilera, copyright (C) Gadi Oxman 2012
 *
 * Utility functions.
 *
 * Functions in this file are not used in the main simulation, but
 * are rather at the initialization and final stages to read/write
 * input/output files, etc.
 */
#define _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "life.h"

static int get_cell(unsigned int *g, int row, int col)
{
	if (row < 0 || row >= GRID_ROWS)
		return 0;
	if (col < 0 || col >= GRID_COLS)
		return 0;
	return (g[row*FIFO_LINE_WIDTH+FIFO_HEADER_WIDTH+col/32] >> (31 - (col % 32))) & 1;
}

static void put_cell(unsigned int *g, int row, int col, int val)
{
	int mask = (1 << (31 - (col % 32)));

	if (val)
		g[row * FIFO_LINE_WIDTH+FIFO_HEADER_WIDTH + col/32] |= mask;
	else
		g[row * FIFO_LINE_WIDTH+FIFO_HEADER_WIDTH + col/32] &= ~mask;
}

/*
 * Write output file
 */
void write_output(char *name, unsigned int *grid)
{
	FILE *fp;
	int row, col;

	printf("\nwriting grid to file %s:\n", name);
	fp = fopen(name, "w");
	if (!fp) {
		printf("ERROR: couldn't open output file %s\n", name);
		exit(1);
	}
	if (ROTATE_GRID) {
		for (row = 0; row < GRID_COLS; row++) {
			for (col = 0; col < GRID_ROWS; col++) {
				if (get_cell(grid, col, GRID_COLS - 1 - row)) {
					fprintf(fp, "%d %d\n", row, col);
				}
			}
		}
	} else {
		for (row = 0; row < GRID_ROWS; row++) {
			for (col = 0; col < GRID_COLS; col++) {
				if (get_cell(grid, row, col)) {
					fprintf(fp, "%d %d\n", row, col);
				}
			}
		}
	}
	fclose(fp);

}

/*
 * Write only chunk of the output file (used in SCC)
 */
void write_output_chunk(char *name, unsigned int *grid, int sw, int ew)
{
	FILE *fp;
	int row, col;

	//	printf("\nwriting grid to file %s, lines %d to %d:\n", name, sw, ew - 1);
	fp = fopen(name, "a");
	if (!fp) {
		printf("ERROR: couldn't open output file %s\n", name);
		exit(1);
	}
#if 1
	if (ROTATE_GRID) {
		// this will actually be printed by collumns - correct but different order from the non rotated
		for (row = 0; row < GRID_COLS; row++) {
			for (col = sw; col < ew; col++) {
				if (get_cell(grid, col, GRID_COLS - 1 - row)) {
					fprintf(fp, "%d %d\n", row, col);
				}
			}
		}
	} else {
		for (row = sw; row < ew; row++) {
			for (col = 0; col < GRID_COLS; col++) {
				if (get_cell(grid, row, col)) {
					fprintf(fp, "%d %d\n", row, col);
				}
			}
		}
	}
#else
	fprintf(fp, "rows %d to %d\n", sw, ew - 1);
#endif
	fclose(fp);

}

/*
 * Read input file, or generate random grid data.
 */
void read_input(char *name, unsigned int *grid)
{
	FILE *fp;
	int row, col, tmp, i;

	memset(grid, 0, GRID_ROWS * FIFO_LINE_WIDTH * sizeof(int));
	for (row = 0; row < GRID_ROWS; row++)
		for (col = 0; col < FIFO_HEADER_WIDTH; col++)
			grid[row*FIFO_LINE_WIDTH+col] = 0xffffffff;

	if (strcmp(name, "random") == 0) {
		//		printf("\ngenerating random grid:\n");
		if (ROTATE_GRID) {
			for (row = 0; row < GRID_COLS; row++) {
				for (col = 0; col < GRID_ROWS/32; col++) {
					tmp = random();
					for (i = 0; i <= 31; i++)
						put_cell(grid, col * 32 + i, GRID_COLS - 1 - row, (tmp >> (31 - i)) & 1);
				}
			}
		} else {
			for (row = 0; row < GRID_ROWS; row++) {
				for (col = 0; col < GRID_COLS32; col++) {
					grid[row*FIFO_LINE_WIDTH+FIFO_HEADER_WIDTH+col] = random();
				}
			}
		}
		char name2[80];
		sprintf(name2, "random_%d_%d.txt", GRID_COLS, GRID_ROWS);
		write_output(name2, grid);
	} else {
		//		printf("\nreading input file: %s\n", name);
		fp = fopen(name, "r");
		if (!fp) {
			printf("ERROR: couldn't open input file %s\n", name);
			exit(1);
		}
		while (!feof(fp)) {
			if (fscanf(fp, "%d %d\n", &row, &col) != 2) {
				printf("ERROR in input file: wrong format\n");
				exit(1);
			}
			if (ROTATE_GRID) {
				tmp = row;
				row = col;
				col = GRID_COLS - 1 - tmp;
			}
			if (row < 0 || row >= GRID_ROWS) {
				printf("ERROR in input file: row %d too large for %dx%d grid\n", row, GRID_COLS, GRID_ROWS);
				exit(1);
			}
			if (col < 0 || col >= GRID_COLS) {
				printf("ERROR in input file: col %d too large for %dx%d grid\n", row, GRID_COLS, GRID_ROWS);
				exit(1);
			}
			put_cell(grid, row, col, 1);
		}
		fclose(fp);
	}
}
