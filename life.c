/*
 * Game Of Life on Tilera, copyright (C) Gadi Oxman 2012
 *
 * See the accompanying PDF for explanation of the algorithms.
 */
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <string.h>
#include <sched.h>

/*
 * Tilera includes
 */
#ifdef TILERA_PLATFORM
#include <tmc/cpus.h>
#include <tmc/task.h>
#include <tmc/spin.h>
#include <tmc/alloc.h>
#include <tmc/udn.h>
#include <arch/cycle.h>
#include <arch/spr.h>
#include <arch/atomic.h>
#include <arch/sim.h>
#endif

/* uncomment for OPENMP SCC RCCE emulator */
//#define restrict /**/

/*
 * SCC includes
 */
#ifdef SCC_PLATFORM
#define MPB_DEBUG 0
#include "RCCE.h"
#endif

#include "life.h"
#include "utility.h"

// max supported number of tiles
#define MAX_TILES 62

/*
 * Our supported algorithms
 */
#define ALGORITHM_TIME_PIPELINE_UDN 0
#define ALGORITHM_TIME_PIPELINE_2STEPS_UDN 1
#define ALGORITHM_TIME_PIPELINE_3STEPS_UDN 2
#define ALGORITHM_SPACE_DIVISION_COHERENT 3
#define ALGORITHM_SPACE_DIVISION_DISTRIBUTED_UDN_MPB 4
#define ALGORITHM_TIME_PIPELINE_MEMFIFO 5
#define ALGORITHM_TIME_PIPELINE_2STEPS_MEMFIFO 6
#define ALGORITHM_TIME_PIPELINE_2LINES_MEMFIFO 7

/*
 * Grid double buffer and barrier for coherent algorithm (not used in distributed algorithms).
 */
unsigned int coherent_grid[2][GRID_ROWS*FIFO_LINE_WIDTH];
#ifdef TILERA_PLATFORM
tmc_spin_barrier_t coherent_barrier;
#endif
#ifdef PC_PLATFORM
pthread_barrier_t coherent_barrier;
#endif
unsigned int coherent_zeros[FIFO_LINE_WIDTH];

/*
 * per tile private data structure
 * each is homed to the local L1 of the tile
 */
typedef struct tile_data_s {
	/*
	 * A fifo holds a section of the grid in a raster scan fashion.
	 * It connects two tiles, homed on the tile which does the reading.
	 * The tile fifos are connected in a circle, passing data from one
	 * to the next, where the last tile returns back the data to tile #0.
	 *
	 * Each line begins with a header, followed by the line cells,
	 * with a 1 bit per cell encoding (1 alive, 0 dead).
	 *
	 * The header encodes information about modified groups of
	 * cells vs the previous generation, 1 bit for each 32 cells.
	 *
	 * This tile reads form this fifo, writes to next one
	 */
#if defined(TILERA_PLATFORM) || defined(PC_PLATFORM)
	unsigned int fifo[FIFO_SIZE];
#endif
#ifdef SCC_PLATFORM
	volatile unsigned int *fifo;
	volatile unsigned int *mpb_fifo_wa;
	volatile unsigned int *mpb_fifo_ra;
	/*
	unsigned int *fifo;
	unsigned int *mpb_fifo_wa;
	unsigned int *mpb_fifo_ra;
	*/


	RCCE_FLAG mpb_flag0, mpb_flag1, mpb_flag0a, mpb_flag1a;
	unsigned int *mpb_buff;
#endif

	unsigned int private_grid[2][GRID_ROWS*FIFO_LINE_WIDTH];
	unsigned int zeros[3*FIFO_LINE_WIDTH];

	// tile id, and prev/next tile ids
	int id;
	int prev_id, next_id;

	// pointer to next/prev tiles in the circular chain
	struct tile_data_s *next, *prev;

	/*
	 * Our fifo read address, updated by us
	 */
	unsigned int fifo_ra;

	/*
	 * Copy of next fifo write address, updated by us
	 */
	unsigned int next_fifo_wa;

	/*
	 * Our fifo write address, updated by the previous tile
	 */
	volatile unsigned int fifo_wa;

	/*
	 * Copy of next fifo read address, updated for us by the next
	 * tile whenever it consumes a line, to avoid polling over the network.
	 */
	volatile unsigned int next_fifo_ra;

#ifdef TILERA_PLATFORM
	/*
	 * Tilera Linux tile ID, this and the next one
	 */
	int tilera_tile_id;

	int next_tilera_tile_id, prev_tilera_tile_id;
	DynamicHeader next_header, prev_header;
	unsigned int next_header_word, prev_header_word;
#endif

	int wrote_output_grid;
} tile_data_t;

// pointers to all tile data, used during configuration to set up next/prev pointers
tile_data_t *tile_data_ptrs[MAX_TILES];

/*****************************************************************************
 * Misc macros
 *****************************************************************************/
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

/*****************************************************************************
 * Parameters
 *****************************************************************************/

/*
 * param_input_file_name: supplied by user, or the keyword "random" for random data.
 * param_output_file_name: supplied by the user.
 * param_algorithm: see defines
 * param_num_steps: number of generations to simulate.
 * param_num_tiles: number of parallel tiles to use.
 */
char *param_input_file_name = "input.txt";
char *param_output_file_name = "output.txt";
int param_max_overlap = 0;
int param_algorithm = ALGORITHM_TIME_PIPELINE_UDN;
int param_num_steps = NUM_STEPS;
int param_num_tiles = NUM_TILES;

/*****************************************************************************
 * stopwatch
 *****************************************************************************/
#ifdef TILERA_PLATFORM
uint64_t start_cc = 0, stop_cc = 0;
static void stopwatch_start(void)
{
	start_cc = get_cycle_count();
	sim_set_tracing(SIM_TRACE_ALL);
	//		sim_profiler_enable();
}
static void stopwatch_stop(void)
{
	stop_cc = get_cycle_count();
	sim_set_tracing(SIM_TRACE_NONE);
	//	sim_profiler_disable();
}
#endif
#if defined(PC_PLATFORM) || defined(SCC_PLATFORM)
struct timeval time_start, time_end;
static void stopwatch_start(void)
{
	gettimeofday(&time_start, NULL);
}

static void stopwatch_stop(void)
{
	gettimeofday(&time_end, NULL);
}
#endif

/*****************************************************************************
 * barrier
 *****************************************************************************/
#ifdef TILERA_PLATFORM
#define barrier_wait() do {tmc_spin_barrier_wait(&coherent_barrier);} while (0)
#endif
#ifdef PC_PLATFORM
#define barrier_wait() do {pthread_barrier_wait(&coherent_barrier);} while (0)
#endif
#ifdef SCC_PLATFORM
#define barrier_wait() do {RCCE_barrier(&RCCE_COMM_WORLD);} while (0)
#endif


/*****************************************************************************
 * Bit parallel macros
 *****************************************************************************/
/*
 * 32-bit parallel halfadder and fulladder
 */
#define HALFADDER(s0,s1,a0,a1) do {	\
		s1 = (a0) & (a1);	\
		s0 = (a0) ^ (a1);	\
	} while(0)

#if 0
#define FULLADDER(s0,s1,a0,a1,a2) do {		\
		unsigned int c0, c1;		\
		HALFADDER(s0,c0,a0,a1);		\
		HALFADDER(s0,c1,s0,a2);		\
		s1 = (c0) | (c1);		\
	} while(0)
#endif
#if 1
#define FULLADDER(s0,s1,a0,a1,a2) do {				\
		s1 = ((a0) & (a1)) | ((a2) & ((a0) ^ (a1)));	\
		s0 = (a2) ^ ((a0) ^ (a1));			\
	} while(0)
#endif

#define ADDER_2BIT_2BIT(xs0,xs1,xs2,xa0,xa1,xb0,xb1) do {		\
		unsigned int xc0;				\
		HALFADDER(xs0,xc0,xa0,xb0);				\
		FULLADDER(xs1,xs2,xa1,xb1,xc0);			\
	} while(0)

#define ADDER_3BIT_2BIT(xs0,xs1,xs2,xs3,xa0,xa1,xa2,xb0,xb1) do {	\
		unsigned int xc0,xc1;				\
		HALFADDER(xs0,xc0,xa0,xb0);				\
		FULLADDER(xs1,xc1,xa1,xb1,xc0);			\
		HALFADDER(xs2,xs3,xa2,xc1);				\
	} while(0)

/*	
 * Calculate the next generation for 32 horizontal cells simultaneously
 * in a bit parallel fashion. This macro is based on Tony Finch
 * at http://dotat.at./prog/life/life.html
 */
#define CELL32V1(output, top_left, top, top_right, left, cur, right,bot_left, bot, bot_right) do { \
		unsigned int sum_left_b1, sum_left_b0;			\
		unsigned int sum_cur_b1, sum_cur_b0;			\
		unsigned int sum_right_b1, sum_right_b0;		\
		unsigned int newone, newtwo, new4a, new4b;		\
		FULLADDER(sum_left_b0,sum_left_b1,			\
			  (top_left << 31) | (top >> 1),		\
			  (left << 31) | (cur >> 1),			\
			  (bot_left << 31) | (bot >> 1));		\
		HALFADDER(sum_cur_b0, sum_cur_b1,			\
			  top,						\
			  bot);						\
		FULLADDER(sum_right_b0,sum_right_b1,			\
			  (top_right >> 31) | (top << 1),		\
			  (right >> 31) | (cur << 1),			\
			  (bot_right >> 31) | (bot << 1));		\
		FULLADDER(newone, newtwo,				\
			  sum_right_b0,					\
			  sum_cur_b0,					\
			  sum_left_b0);					\
		FULLADDER(newtwo, new4a,				\
			  newtwo,					\
			  sum_right_b1,					\
			  sum_left_b1);					\
		HALFADDER(newtwo,new4b,					\
			  newtwo,					\
			  sum_cur_b1);					\
		newone = newone | cur;					\
		output = newone & newtwo & ~new4a & ~new4b;		\
	} while(0)

#define CELL32V2(output, top_left, top, top_right, left, cur, right,bot_left, bot, bot_right) do { \
		unsigned int sum_top_b1, sum_top_b0;			\
		unsigned int sum_cur_b1, sum_cur_b0;			\
		unsigned int sum_bot_b1, sum_bot_b0;			\
		unsigned int newone, newtwo, new4a, new4b;		\
		FULLADDER(sum_top_b0,sum_top_b1,			\
			  (top_left << 31) | (top >> 1),		\
			  top,						\
			  (top_right >> 31) | (top << 1));		\
		HALFADDER(sum_cur_b0, sum_cur_b1,			\
			  (left << 31) | (cur >> 1),			\
			  (right >> 31) | (cur << 1));			\
		FULLADDER(sum_bot_b0,sum_bot_b1,			\
			  (bot_left << 31) | (bot >> 1),		\
			  bot,						\
			  ((bot_right >> 31) | (bot << 1)));		\
		FULLADDER(newone, newtwo,				\
			  sum_bot_b0,					\
			  sum_cur_b0,					\
			  sum_top_b0);					\
		FULLADDER(newtwo, new4a,				\
			  newtwo,					\
			  sum_bot_b1,					\
			  sum_top_b1);					\
		HALFADDER(newtwo,new4b,					\
			  newtwo,					\
			  sum_cur_b1);					\
		newone = newone | cur;					\
		output = newone & newtwo & ~new4a & ~new4b;		\
	} while(0)

#define CELL32V3(output, top_left, top, top_right, left, cur, right,bot_left, bot, bot_right) do { \
		unsigned int sum_top_b1, sum_top_b0;			\
		unsigned int sum_cur_b1, sum_cur_b0;			\
		unsigned int sum_bot_b1, sum_bot_b0;			\
		unsigned int s0, s1, s2, s3, t1, c1, c2_1, c2_2;	\
		FULLADDER(sum_top_b0,sum_top_b1,			\
			  (top_left << 31) | (top >> 1),		\
			  top,						\
			  (top_right >> 31) | (top << 1));		\
		FULLADDER(sum_cur_b0, sum_cur_b1,			\
			  (left << 31) | (cur >> 1),			\
			  cur,						\
			  (right >> 31) | (cur << 1));			\
		FULLADDER(sum_bot_b0,sum_bot_b1,			\
			  (bot_left << 31) | (bot >> 1),		\
			  bot,						\
			  ((bot_right >> 31) | (bot << 1)));		\
		FULLADDER(s0, c1,					\
			  sum_top_b0,					\
			  sum_cur_b0,					\
			  sum_bot_b0);					\
		FULLADDER(t1, c2_1,					\
			  sum_top_b1,					\
			  sum_cur_b1,					\
			  sum_bot_b1);					\
		HALFADDER(s1, c2_2,					\
			  c1,						\
			  t1);						\
		HALFADDER(s2, s3,					\
			  c2_1,						\
			  c2_2);					\
		output = ~s3 & ((~s2 & s1 & s0) | (cur & s2 & ~s1 & ~s0)); \
	} while(0)

#define CELL32V5_1(output, cur, i0, i1, i2, i3, i4, i5, i6, i7) do {	\
		int s01,s23,s45,s67;					\
		int c01,c23,c45,c67;					\
		int c0123_0,c0123_1,s0123_0,s0123_1;			\
		int c4567_0,c4567_1,s4567_0,s4567_1;			\
		int sum0, sum1, ca0, ca1;				\
		HALFADDER(s01,c01,i0,i1);				\
		HALFADDER(s23,c23,i3,i4);				\
		HALFADDER(s45,c45,i2,i5);				\
		HALFADDER(s67,c67,i6,i7);				\
		HALFADDER(s0123_0,c0123_0,s01,s23);			\
		HALFADDER(s4567_0,c4567_0,s45,s67);			\
		FULLADDER(s0123_1,c0123_1,c01,c23,c0123_0);		\
		FULLADDER(s4567_1,c4567_1,c45,c67,c4567_0);		\
		HALFADDER(sum0,ca0,s4567_0,s0123_0);			\
		FULLADDER(sum1,ca1,s4567_1,s0123_1,ca0);		\
		output = (sum0 | cur) & sum1 & ~(c4567_1|c0123_1|ca1);	\
	} while(0)
//		int sum0, sum1, sum2, ca0, ca1, ca2;
//		FULLADDER(sum2,ca2,c4567_1,c0123_1,ca1);


#define CELL32V5(output, top_left, top, top_right, left, cur, right,bot_left, bot, bot_right) \
	CELL32V5_1(output, cur,						\
			    (top_left << 31) | (top >> 1),		\
			    (left << 31) | (cur >> 1),			\
			    (bot_left << 31) | (bot >> 1),		\
			    top,					\
			    bot,					\
			    (top_right >> 31) | (top << 1),		\
			    (right >> 31) | (cur << 1),			\
			    (bot_right >> 31) | (bot << 1))

#define CELL32HOR(b0, b1, left, cur, right) do {			\
		FULLADDER(b0, b1,					\
			  (left << 31) | (cur >> 1),			\
			  cur,						\
			  (right >> 31) | (cur << 1));			\
	} while(0)

#define CELL32VER(output, cur, sum_top_b0, sum_top_b1, sum_cur_b0, sum_cur_b1, sum_bot_b0, sum_bot_b1) do { \
		unsigned int s0, s1, s2, s3, t1, c1, c2_1, c2_2;	\
		FULLADDER(s0, c1,					\
			  sum_top_b0,					\
			  sum_cur_b0,					\
			  sum_bot_b0);					\
		FULLADDER(t1, c2_1,					\
			  sum_top_b1,					\
			  sum_cur_b1,					\
			  sum_bot_b1);					\
		HALFADDER(s1, c2_2,					\
			  c1,						\
			  t1);						\
		HALFADDER(s2, s3,					\
			  c2_1,						\
			  c2_2);					\
		output = ~s3 & ((~s2 & s1 & s0) | (cur & s2 & ~s1 & ~s0)); \
	} while(0)

#define CELL32 CELL32V2

#define CELL32M(input_modified, output_modified, output, top_left, top, top_right, left, cur, right,bot_left, bot, bot_right) do { \
		if (input_modified) {					\
			CELL32(output,					\
			       top_left, top, top_right,		\
			       left, cur, right,			\
			       bot_left, bot, bot_right);		\
			line_modified = (line_modified << 1) | (output != cur);	\
		} else {						\
			output = cur;					\
			line_modified = line_modified << 1;		\
		}							\
	} while (0)


/*	
 * 64 bit version for 2 vertical lines of 32 cells each
 */
#define CELL64(output0, output1, top_left, top, top_right, left0, cur0, right0, left1, cur1, right1, bot_left, bot, bot_right) do { \
		int sum_top_b1, sum_top_b0;				\
		int sum_cur0_b1, sum_cur0_b0;				\
		int sum_cur1_b1, sum_cur1_b0;				\
		int sum_cur01_b2, sum_cur01_b1, sum_cur01_b0;		\
		int sum_bot_b1, sum_bot_b0;				\
		int sum0_b3,sum0_b2,sum0_b1,sum0_b0;			\
		int sum1_b3,sum1_b2,sum1_b1,sum1_b0;			\
		FULLADDER(sum_cur0_b0, sum_cur0_b1,			\
			  (left0 << 31) | (cur0 >> 1),			\
			  cur0,						\
			  (right0 >> 31) | (cur0 << 1));			\
		FULLADDER(sum_cur1_b0, sum_cur1_b1,			\
			  (left1 << 31) | (cur1 >> 1),			\
			  cur1,						\
			  (right1 >> 31) | (cur1 << 1));		\
		ADDER_2BIT_2BIT(sum_cur01_b0,sum_cur01_b1,sum_cur01_b2,	\
			    sum_cur0_b0,sum_cur0_b1,			\
			    sum_cur1_b0,sum_cur1_b1);			\
		FULLADDER(sum_top_b0, sum_top_b1,			\
			  (top_left << 31) | (top >> 1),		\
			  top,						\
			  (top_right >> 31) | (top << 1));		\
		FULLADDER(sum_bot_b0, sum_bot_b1,			\
			  (bot_left << 31) | (bot >> 1),		\
			  bot,						\
			  (bot_right >> 31) | (bot << 1));		\
		ADDER_3BIT_2BIT(sum0_b0,sum0_b1,sum0_b2,sum0_b3,	\
				sum_cur01_b0,sum_cur01_b1,sum_cur01_b2,	\
				sum_top_b0, sum_top_b1);		\
		ADDER_3BIT_2BIT(sum1_b0,sum1_b1,sum1_b2,sum1_b3,	\
				sum_cur01_b0,sum_cur01_b1,sum_cur01_b2,	\
				sum_bot_b0, sum_bot_b1);		\
		output0 = ~sum0_b3 & ((~sum0_b2 & sum0_b1 & sum0_b0) | (cur0 & sum0_b2 & ~sum0_b1 & ~sum0_b0));	\
		output1 = ~sum1_b3 & ((~sum1_b2 & sum1_b1 & sum1_b0) | (cur1 & sum1_b2 & ~sum1_b1 & ~sum1_b0));	\
	} while(0)

/*****************************************************************************
 * FIFO functions
 *****************************************************************************/
static inline void fifo_wait(tile_data_t *td, int wa, int our_full_threshold, int next_space_threshold)
{
	int our_fifo_fullness, next_fifo_space, next_fifo_fullness;
	for (;;) {
		our_fifo_fullness = (td->fifo_wa - td->fifo_ra + FIFO_SIZE);
		if (our_fifo_fullness >= FIFO_SIZE)
			our_fifo_fullness -= FIFO_SIZE;
		next_fifo_fullness = (wa - td->next_fifo_ra + FIFO_SIZE);
		if (next_fifo_fullness >= FIFO_SIZE)
			next_fifo_fullness -= FIFO_SIZE;
		next_fifo_space = FIFO_SIZE - next_fifo_fullness - 1;
		if (our_fifo_fullness >= our_full_threshold && next_fifo_space >= next_space_threshold)
			break;
	}
}

#ifdef SCC_PLATFORM
#include "RCCE_lib.h"
#include "RCCE_memcpy.c"

//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_get_char
//--------------------------------------------------------------------------------------
// copy one byte from address "source" in the remote MPB to address "target" in either 
// the local MPB, or in the calling UE's private memory. 
//--------------------------------------------------------------------------------------
int RCCE_get_int(
		 volatile unsigned int *target, // target buffer, MPB or private memory
		 volatile unsigned int *source, // source buffer, MPB
		 int ID           // rank of source UE
		 ) {

	// in non-GORY mode we only need to retain the MPB source shift; we
	// already know the source is in the MPB, not private memory
	source = (volatile unsigned int *) RCCE_comm_buffer[ID]+(source-(volatile unsigned int *)RCCE_comm_buffer[RCCE_IAM]);

#ifdef _OPENMP
	// make sure that any data that has been put in our MPB by another UE is visible 
#pragma omp flush
#endif

	// do the actual copy 
	RC_cache_invalidate();
	//	printf("get_int: target %p, source %p\n", target, source);
	*target = *source;
#ifdef _OPENMP
	// flush data to make sure it is visible to all threads; cannot use a flush list 
	// because it concerns malloced space                     
#pragma omp flush
#endif
	return(RCCE_SUCCESS);
}

//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_get
//--------------------------------------------------------------------------------------
// copy data from address "source" in the remote MPB to address "target" in either the
// local MPB, or in the calling UE's private memory. We do not test to see if a move
// into the calling UE's private memory stays within allocated memory                     *
//--------------------------------------------------------------------------------------
int RCCE_get_fast(
  t_vcharp target, // target buffer, MPB or private memory
  t_vcharp source, // source buffer, MPB
  int num_bytes,   // number of bytes to copy (must be multiple of cache line size
  int ID           // rank of source UE
  ) {

    // in non-GORY mode we only need to retain the MPB source shift; we
    // already know the source is in the MPB, not private memory
    source = RCCE_comm_buffer[ID]+(source-RCCE_comm_buffer[RCCE_IAM]);


#ifdef _OPENMP
  // make sure that any data that has been put in our MPB by another UE is visible 
  #pragma omp flush
#endif

  // do the actual copy 
  RC_cache_invalidate();
#ifdef SCC
  memcpy_get((void *)target, (void *)source, num_bytes);
#else
  memcpy((void *)target, (void *)source, num_bytes);
#endif

#ifdef _OPENMP
  // flush data to make sure it is visible to all threads; cannot use a flush list 
  // because it concerns malloced space                     
  #pragma omp flush
#endif
  return(RCCE_SUCCESS);
}


//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_put
//--------------------------------------------------------------------------------------
// copy data from address "source" in the local MPB or the calling UE's private memory 
// to address "target" in the remote MPB. We do not test to see if a move from the 
// calling UE's private memory stays within allocated memory                        
//--------------------------------------------------------------------------------------
int RCCE_put_fast(
  t_vcharp target, // target buffer, MPB
  t_vcharp source, // source buffer, MPB or private memory
  int num_bytes, 
  int ID
  ) {

  // in non-GORY mode we only need to retain the MPB target shift; we
  // already know the target is in the MPB, not private memory
  target = RCCE_comm_buffer[ID]+(target-RCCE_comm_buffer[RCCE_IAM]);    

#ifdef _OPENMP
  // make sure that any data that has been put in our MPB by another UE is visible 
  #pragma omp flush
#endif

  // do the actual copy 
//  RC_cache_invalidate();
#ifdef SCC
  memcpy_put((void *)target, (void *)source, num_bytes);
#else
  memcpy((void *)target, (void *)source, num_bytes);
#endif

#ifdef _OPENMP
  // flush data to make it visible to all threads; cannot use flush list because it 
  // concerns malloced space                        
  #pragma omp flush
#endif
  return(RCCE_SUCCESS);
}

//--------------------------------------------------------------------------------------
// FUNCTION: RCCE_put_int
//--------------------------------------------------------------------------------------
// copy one byte from address "source" in the local MPB or the calling UE's private 
// memory to address "target" in the remote MPB. 
//--------------------------------------------------------------------------------------
int RCCE_put_int(
		 volatile unsigned int *target, // target buffer, MPB
		 volatile unsigned int *source, // source buffer, MPB or private memory
		 int ID,
		 int flush
  ) {

  // in non-GORY mode we only need to retain the MPB target shift; we
  // already know the target is in the MPB, not private memory
	target = (volatile unsigned int *) RCCE_comm_buffer[ID]+(target-(volatile unsigned int *) RCCE_comm_buffer[RCCE_IAM]);    

#ifdef _OPENMP
  // make sure that any data that has been put in our MPB by another UE is visible 
  #pragma omp flush
#endif

  // do the actual copy 
//  RC_cache_invalidate();
  *target = *source;
#ifdef _OPENMP
  // flush data to make it visible to all threads; cannot use flush list because it 
  // concerns malloced space                        
  #pragma omp flush
#else
  if (flush) {
	  // need to write to another line to make sure the write combine buffer gets flushed
	  *(int *)RCCE_fool_write_combine_buffer = 1;
  }
#endif
  return(RCCE_SUCCESS);
}
#endif

static inline void fifo_wait_not_empty(tile_data_t *td)
{
	while (td->fifo_wa == td->fifo_ra) {
#ifdef SCC_PLATFORM
		// update our fifo_wa from on chip MPB memory
		RCCE_get_int(&td->fifo_wa, td->mpb_fifo_wa, td->id);
#if MPB_DEBUG
		printf("tile %d: wait_fullness: updating fifo wa from our MPB: %d\n", td->id, td->fifo_wa);
#endif
#endif
	}
}

static inline void fifo_wait_not_full(tile_data_t *td)
{
	int next_fifo_space, next_fifo_fullness;
	int wa = td->next_fifo_wa;
	int ra = td->next_fifo_ra;

	next_fifo_fullness = (wa - ra + FIFO_SIZE);
	if (next_fifo_fullness >= FIFO_SIZE)
		next_fifo_fullness -= FIFO_SIZE;
	next_fifo_space = FIFO_SIZE - next_fifo_fullness - 1;
	if (next_fifo_space >= FIFO_LINE_WIDTH)
		return;

	while (td->next_fifo_ra == ra) {
#ifdef SCC_PLATFORM
		// update our copy of next fifo_ra from our chip MPB memory
		RCCE_get_int(&td->next_fifo_ra, td->mpb_fifo_ra, td->id);
#if MPB_DEBUG
		printf("tile %d: wait_space: updating next_fifo_ra from our MPB: %d\n", td->id, td->next_fifo_ra);
#endif
#endif
	}
}

static inline int fifo_prev_line(int address)
{
	int prev_address = address - FIFO_LINE_WIDTH;
	if (prev_address < 0)
		prev_address += FIFO_SIZE;
	return prev_address;
}

static inline int fifo_next_line(int address)
{
	int next_address = address + FIFO_LINE_WIDTH;
	if (next_address >= FIFO_SIZE)
		next_address -= FIFO_SIZE;
	return next_address;
}

static inline void update_next_tile_fifo_wa(tile_data_t *td, int wa)
{
#ifdef TILERA_PLATFORM
	atomic_write_barrier();
#endif
	td->next_fifo_wa = wa;
#if defined(TILERA_PLATFORM) || defined(PC_PLATFORM)
	td->next->fifo_wa = wa;
#endif
#ifdef SCC_PLATFORM
#if MPB_DEBUG
	printf("tile %d: update_next_tile_fifo_wa, %d\n", td->id, wa);
#endif
	RCCE_put_int(td->mpb_fifo_wa, (volatile unsigned int *) &wa, td->next_id, 1);
#endif
}

static void update_prev_tile_fifo_ra(tile_data_t *td, int ra)
{
	td->fifo_ra = ra;
#if defined(TILERA_PLATFORM) || defined(PC_PLATFORM)
	td->prev->next_fifo_ra = ra;
#endif
#ifdef SCC_PLATFORM
#if MPB_DEBUG
	printf("tile %d: update_prev_tile_fifo_ra, %d\n", td->id, ra);
#endif
	RCCE_put_int(td->mpb_fifo_ra, (volatile unsigned int *) &ra, td->prev_id, 1);
#endif
}


#ifdef TILERA_PLATFORM

/*****************************************************************************
 * UDN functions
 *****************************************************************************/
/*
 * Send grid line over UDN to the next tile (uses udn0)
 */
static inline void send_udn_cells_next(tile_data_t *td, unsigned int *p)
{
	int i;

	__tile_udn_send(td->next_header_word);
	__tile_udn_send(TMC_UDN0_DEMUX_TAG);
	for (i = 0; i < GRID_COLS32; i++)
		__tile_udn_send(p[FIFO_HEADER_WIDTH+i]);
	__tile_udn_send(p[0]);
}

/*
 * Receive grid line over UDN from the previous tile (uses udn0)
 */
static inline void receive_udn_cells_prev(tile_data_t *td, unsigned int *p)
{
	int i;

	for (i = 0; i < GRID_COLS32; i++)
		p[FIFO_HEADER_WIDTH+i] = __tile_udn0_receive();
	p[0] = __tile_udn0_receive();
}

/*
 * Send grid line over UDN to the prev tile (uses udn1)
 */
static inline void send_udn_cells_prev(tile_data_t *td, unsigned int *p)
{
	int i;

	__tile_udn_send(td->prev_header_word);
	__tile_udn_send(TMC_UDN1_DEMUX_TAG);
	for (i = 0; i < GRID_COLS32; i++)
		__tile_udn_send(p[FIFO_HEADER_WIDTH+i]);
	__tile_udn_send(p[0]);
}

/*
 * Receive grid line over UDN from the previous tile (uses udn1)
 */
static inline void receive_udn_cells_next(tile_data_t *td, unsigned int *p)
{
	int i;

	for (i = 0; i < GRID_COLS32; i++)
		p[FIFO_HEADER_WIDTH+i] = __tile_udn1_receive();
	p[0] = __tile_udn1_receive();
}
#endif /* TILERA_PLATFORM */

/*
 * Receive cells over the UDN (Tilera) / MPB (Intel SCC)
 */
static inline void receive_cells(tile_data_t *td, unsigned int *p, int row)
{
#ifdef TILERA_PLATFORM
	receive_udn_cells_prev(td, p);
#endif
#if defined(PC_PLATFORM) || defined(SCC_PLATFORM)
	int ra;

	fifo_wait_not_empty(td);

	ra = td->fifo_ra;
#ifdef SCC_PLATFORM
	RCCE_get_fast((t_vcharp) p, (t_vcharp) (td->fifo + ra), FIFO_LINE_WIDTH * sizeof(int), td->id);
#else
	int i;
	for (i = 0; i < FIFO_LINE_WIDTH; i++)
		p[i] = td->fifo[ra+i];
#endif
	ra = fifo_next_line(ra);
	update_prev_tile_fifo_ra(td, ra);
#endif
}

static inline void send_line_start(tile_data_t *td)
{
#ifdef TILERA_PLATFORM
	__tile_udn_send(td->next_header_word);
	__tile_udn_send(TMC_UDN0_DEMUX_TAG);
#endif
#if defined(PC_PLATFORM) || defined(SCC_PLATFORM)
	fifo_wait_not_full(td);
#endif
}

static inline void send_cell(tile_data_t *td, unsigned int cell, int idx)
{
#ifdef TILERA_PLATFORM
	__tile_udn_send(cell);
#endif
#ifdef SCC_PLATFORM
	RCCE_put_int((volatile unsigned int *) (td->fifo + td->next_fifo_wa + FIFO_HEADER_WIDTH + idx), &cell, td->next_id,0);
#endif
#ifdef PC_PLATFORM
	td->next->fifo[td->next_fifo_wa+FIFO_HEADER_WIDTH+idx] = cell;
#endif
}

static inline void send_line_end(tile_data_t *td, int line_modified)
{
#ifdef TILERA_PLATFORM
	__tile_udn_send(line_modified);
#endif
#ifdef PC_PLATFORM
	td->next->fifo[td->next_fifo_wa+0] = line_modified;
#endif
#ifdef SCC_PLATFORM
	RCCE_put_int((volatile unsigned int *) (td->fifo + td->next_fifo_wa + 0), &line_modified, td->next_id,0);
#endif
#if defined(PC_PLATFORM) || defined(SCC_PLATFORM)
	update_next_tile_fifo_wa(td, fifo_next_line(td->next_fifo_wa));
#endif
}

/*
 * Send cells over the UDN (Tilera) / MPB (Intel SCC)
 */
static inline void send_cells(tile_data_t *td, unsigned int *p, int row)
{
#ifdef TILERA_PLATFORM
	send_udn_cells_next(td, p);
#endif
#if defined(PC_PLATFORM) || defined(SCC_PLATFORM)
	int wa;

	wa = td->next_fifo_wa;
	fifo_wait_not_full(td);
#ifdef SCC_PLATFORM
	RCCE_put_fast((t_vcharp) (td->fifo + wa), (t_vcharp) p, FIFO_LINE_WIDTH * sizeof(int), td->next_id);
#else
	unsigned int *output_fifo;
	int i;

	output_fifo = td->next->fifo;
	//	memcpy(output_fifo + wa, p, FIFO_LINE_WIDTH * sizeof(int));
	for (i = 0; i < FIFO_LINE_WIDTH; i++)
		output_fifo[wa+i] = p[i];
#endif
#if MPB_DEBUG
	printf("tile %d, send_cells, row %d, wa %d\n", td->id, row, wa);
#endif
	wa = fifo_next_line(wa);
	update_next_tile_fifo_wa(td, wa);
#endif
}

static inline void send_cells_next(tile_data_t *td, unsigned int *p, int nr_rows)
{
#ifdef TILERA_PLATFORM
	while (nr_rows--) {
		send_udn_cells_next(td, p);
		p += FIFO_LINE_WIDTH;
	}
#endif
#ifdef SCC_PLATFORM
	RCCE_wait_until(td->mpb_flag0a, RCCE_FLAG_SET);
	RCCE_flag_write(&td->mpb_flag0a, RCCE_FLAG_UNSET, td->id); 
	RCCE_put((t_vcharp)(&td->mpb_buff[0]), (t_vcharp) p, nr_rows*FIFO_LINE_WIDTH*sizeof(int), td->id+1);
	RCCE_flag_write(&td->mpb_flag0, RCCE_FLAG_SET, td->id+1); 
#endif
}

static inline void send_cells_prev(tile_data_t *td, unsigned int *p, int nr_rows)
{
#ifdef TILERA_PLATFORM
	while (nr_rows--) {
		send_udn_cells_prev(td, p);
		p += FIFO_LINE_WIDTH;
	}
#endif
#ifdef SCC_PLATFORM
		RCCE_wait_until(td->mpb_flag1a, RCCE_FLAG_SET);
		RCCE_flag_write(&td->mpb_flag1a, RCCE_FLAG_UNSET, td->id); 
		RCCE_put((t_vcharp)(&td->mpb_buff[FIFO_LINE_WIDTH*nr_rows]), (t_vcharp) p, nr_rows*FIFO_LINE_WIDTH*sizeof(int), td->id-1);
		RCCE_flag_write(&td->mpb_flag1, RCCE_FLAG_SET, td->id-1); 
#endif
}

static inline void receive_cells_next(tile_data_t *td, unsigned int *p, int nr_rows)
{
#ifdef TILERA_PLATFORM
	while (nr_rows--) {
		receive_udn_cells_next(td, p);
		p += FIFO_LINE_WIDTH;
	}
#endif
#ifdef SCC_PLATFORM
	RCCE_wait_until(td->mpb_flag1, RCCE_FLAG_SET);
	RCCE_flag_write(&td->mpb_flag1, RCCE_FLAG_UNSET, td->id);			
	RCCE_get((t_vcharp) p, (t_vcharp)(&td->mpb_buff[FIFO_LINE_WIDTH*nr_rows]), nr_rows*FIFO_LINE_WIDTH*sizeof(int),td->id);
	RCCE_flag_write(&td->mpb_flag1a, RCCE_FLAG_SET, td->id+1); 
#endif
}

static inline void receive_cells_prev(tile_data_t *td, unsigned int *p, int nr_rows)
{
#ifdef TILERA_PLATFORM
	while (nr_rows--) {
		receive_udn_cells_prev(td, p);
		p += FIFO_LINE_WIDTH;
	}
#endif
#ifdef SCC_PLATFORM
	RCCE_wait_until(td->mpb_flag0, RCCE_FLAG_SET);
	RCCE_flag_write(&td->mpb_flag0, RCCE_FLAG_UNSET, td->id); 
	RCCE_get((t_vcharp) p, (t_vcharp)(&td->mpb_buff[0]), nr_rows*FIFO_LINE_WIDTH*sizeof(int),td->id);
	RCCE_flag_write(&td->mpb_flag0a, RCCE_FLAG_SET, td->id-1); 
#endif
}

/*****************************************************************************
 * Line processing
 *****************************************************************************/
static int process_line(int check_modified, unsigned int * restrict wa, unsigned int * restrict ra_top, unsigned int * restrict ra_cur, unsigned int * restrict ra_bot)
{
	int col, cells_modified, line_modified;

#if 0
	check_modified = 0;
#endif
	line_modified = 0;

	if (check_modified && (GRID_COLS32 <= 32)) {
		cells_modified = ra_top[0]|ra_cur[0]|ra_bot[0];
		cells_modified = (cells_modified << 1) | (cells_modified) | (cells_modified >> 1);
	} else
		cells_modified = 0xffffffff;

	if (!cells_modified) {
#if 1
		memcpy(wa, ra_cur, FIFO_LINE_WIDTH * sizeof(int));
#else
		for (col = 0; col < GRID_COLS32; col++)
			wa[FIFO_HEADER_WIDTH+col] = ra_cur[FIFO_HEADER_WIDTH+col];
		wa[0] = 0;
#endif
	} else {
		CELL32M(cells_modified & (1 << (GRID_COLS32 - 1)), line_modified,
			wa[FIFO_HEADER_WIDTH+0],
			0, ra_top[FIFO_HEADER_WIDTH+0], ra_top[FIFO_HEADER_WIDTH+1],
			0, ra_cur[FIFO_HEADER_WIDTH+0], ra_cur[FIFO_HEADER_WIDTH+1],
			0, ra_bot[FIFO_HEADER_WIDTH+0], ra_bot[FIFO_HEADER_WIDTH+1]);
		for (col = 1; col < GRID_COLS32 - 1; col++) {
			CELL32M(cells_modified & (1 << (GRID_COLS32 - 1 - col)), line_modified,
				wa[FIFO_HEADER_WIDTH+col],
				ra_top[FIFO_HEADER_WIDTH+col-1], ra_top[FIFO_HEADER_WIDTH+col], ra_top[FIFO_HEADER_WIDTH+col+1],
				ra_cur[FIFO_HEADER_WIDTH+col-1], ra_cur[FIFO_HEADER_WIDTH+col], ra_cur[FIFO_HEADER_WIDTH+col+1],
				ra_bot[FIFO_HEADER_WIDTH+col-1], ra_bot[FIFO_HEADER_WIDTH+col], ra_bot[FIFO_HEADER_WIDTH+col+1]);
		}
		CELL32M(cells_modified & (1 << 0), line_modified,
			wa[FIFO_HEADER_WIDTH+GRID_COLS32-1],
			ra_top[FIFO_HEADER_WIDTH+GRID_COLS32-2], ra_top[FIFO_HEADER_WIDTH+GRID_COLS32-1], 0,
			ra_cur[FIFO_HEADER_WIDTH+GRID_COLS32-2], ra_cur[FIFO_HEADER_WIDTH+GRID_COLS32-1], 0,
			ra_bot[FIFO_HEADER_WIDTH+GRID_COLS32-2], ra_bot[FIFO_HEADER_WIDTH+GRID_COLS32-1], 0);
		wa[0] = line_modified;
	}
	return line_modified;
}

static inline int process_line_and_send(tile_data_t *td, int check_modified, unsigned int * restrict wa, unsigned int * restrict ra_top, unsigned int * restrict ra_cur, unsigned int * restrict ra_bot)
{
	int line_modified = 0;

#if 0
	line_modified = process_line(check_modified, wa, ra_top, ra_cur, ra_bot);
	send_cells(td, wa, 0);
#else
	int col, cells_modified, newcells;

#if 0
	check_modified = 0;
#endif

	if (check_modified && (GRID_COLS32 <= 32)) {
		cells_modified = ra_top[0]|ra_cur[0]|ra_bot[0];
		cells_modified = (cells_modified << 1) | (cells_modified) | (cells_modified >> 1);
	} else
		cells_modified = 0xffffffff;

	if (!cells_modified) {
		send_cells(td, ra_cur, 0);
	} else {
		send_line_start(td);
		CELL32M(cells_modified & (1 << (GRID_COLS32 - 1)), line_modified,
			newcells,
			0, ra_top[FIFO_HEADER_WIDTH+0], ra_top[FIFO_HEADER_WIDTH+1],
			0, ra_cur[FIFO_HEADER_WIDTH+0], ra_cur[FIFO_HEADER_WIDTH+1],
			0, ra_bot[FIFO_HEADER_WIDTH+0], ra_bot[FIFO_HEADER_WIDTH+1]);
		send_cell(td, newcells, 0);
		for (col = 1; col < GRID_COLS32 - 1; col++) {
			CELL32M(cells_modified & (1 << (GRID_COLS32 - 1 - col)), line_modified,
				newcells,
				ra_top[FIFO_HEADER_WIDTH+col-1], ra_top[FIFO_HEADER_WIDTH+col], ra_top[FIFO_HEADER_WIDTH+col+1],
				ra_cur[FIFO_HEADER_WIDTH+col-1], ra_cur[FIFO_HEADER_WIDTH+col], ra_cur[FIFO_HEADER_WIDTH+col+1],
				ra_bot[FIFO_HEADER_WIDTH+col-1], ra_bot[FIFO_HEADER_WIDTH+col], ra_bot[FIFO_HEADER_WIDTH+col+1]);
			send_cell(td, newcells, col);
		}
		CELL32M(cells_modified & (1 << 0), line_modified,
			newcells,
			ra_top[FIFO_HEADER_WIDTH+GRID_COLS32-2], ra_top[FIFO_HEADER_WIDTH+GRID_COLS32-1], 0,
			ra_cur[FIFO_HEADER_WIDTH+GRID_COLS32-2], ra_cur[FIFO_HEADER_WIDTH+GRID_COLS32-1], 0,
			ra_bot[FIFO_HEADER_WIDTH+GRID_COLS32-2], ra_bot[FIFO_HEADER_WIDTH+GRID_COLS32-1], 0);
		send_cell(td, newcells, GRID_COLS32 - 1);
		send_line_end(td, line_modified);
	}
#endif
	return line_modified;
}

static int process_2lines(int check_modified, unsigned int *wa_cur0, unsigned int *wa_cur1, unsigned int *ra_top, unsigned int *ra_cur0, unsigned int *ra_cur1, unsigned int *ra_bot)
{
	unsigned int *top_cells, *cur0_cells, *cur1_cells, *bot_cells, *new0_cells, *new1_cells;
	int col;
	unsigned int cells0_modified, cells1_modified, cellsb_modified, cells01_modified, line0_modified, line1_modified;

#if 0
	check_modified = 0;
#endif

	line0_modified = 0;
	line1_modified = 0;

	if (check_modified && (GRID_COLS32 <= 32)) {
		cells0_modified = ra_top[0]|ra_cur0[0]|ra_cur1[0];
		cells0_modified = (cells0_modified << 1) | (cells0_modified) | (cells0_modified >> 1);
		cells1_modified = ra_cur0[0]|ra_cur1[0]|ra_bot[0];
		cells1_modified = (cells1_modified << 1) | (cells1_modified) | (cells1_modified >> 1);
	} else {
		cells0_modified = 0xffffffff;
		cells1_modified = 0xffffffff;
	}

	cells01_modified = cells0_modified | cells1_modified;
	if (!cells0_modified)
		memcpy(wa_cur0, ra_cur0, FIFO_LINE_WIDTH * sizeof(int));
	if (!cells1_modified)
		memcpy(wa_cur1, ra_cur1, FIFO_LINE_WIDTH * sizeof(int));

	if (cells0_modified && !cells1_modified) {
		line0_modified = process_line(check_modified, wa_cur0, ra_top, ra_cur0, ra_cur1);
		wa_cur0[0] = line0_modified;
	} else if (cells1_modified && !cells0_modified) {
		line1_modified = process_line(check_modified, wa_cur1, ra_cur0, ra_cur1, ra_bot);
		wa_cur1[0] = line1_modified;
	} else if (cells0_modified && cells1_modified) {
		cellsb_modified = cells0_modified & cells1_modified;

		top_cells = ra_top + FIFO_HEADER_WIDTH;
		cur0_cells = ra_cur0 + FIFO_HEADER_WIDTH;
		cur1_cells = ra_cur1 + FIFO_HEADER_WIDTH;
		new0_cells = (unsigned int *) (wa_cur0 + FIFO_HEADER_WIDTH);
		new1_cells = (unsigned int *) (wa_cur1 + FIFO_HEADER_WIDTH);
		bot_cells = ra_bot + FIFO_HEADER_WIDTH;

		if (cells01_modified & (1 << (GRID_COLS32-1)))
			CELL64(new0_cells[0], new1_cells[0],
			       0, top_cells[0], top_cells[1],
			       0, cur0_cells[0], cur0_cells[1],
			       0, cur1_cells[0], cur1_cells[1],
			       0, bot_cells[0], bot_cells[1]);
		else {
			new0_cells[0] = cur0_cells[0];
			new1_cells[0] = cur1_cells[0];
		}

		for (col = 1; col < GRID_COLS32 - 1; col++) {
#if 1
			if (cells01_modified & (1 << (GRID_COLS32 - 1 - col)))
				CELL64(new0_cells[col], new1_cells[col],
				       top_cells[col-1], top_cells[col], top_cells[col+1],
				       cur0_cells[col-1], cur0_cells[col], cur0_cells[col+1],
				       cur1_cells[col-1], cur1_cells[col], cur1_cells[col+1],
				       bot_cells[col-1], bot_cells[col], bot_cells[col+1]);
			else {
				new0_cells[col] = cur0_cells[col];
				new1_cells[col] = cur1_cells[col];
			}
#else
			if (cellsb_modified & (1 << (GRID_COLS32 - 1 - col)))
				CELL64(new0_cells[col], new1_cells[col],
				       top_cells[col-1], top_cells[col], top_cells[col+1],
				       cur0_cells[col-1], cur0_cells[col], cur0_cells[col+1],
				       cur1_cells[col-1], cur1_cells[col], cur1_cells[col+1],
				       bot_cells[col-1], bot_cells[col], bot_cells[col+1]);
			else {
				if (cells0_modified & (1 << (GRID_COLS32 - 1 - col)))
					CELL32(new0_cells[col],
					       top_cells[col-1], top_cells[col], top_cells[col+1],
					       cur0_cells[col-1], cur0_cells[col], cur0_cells[col+1],
					       cur1_cells[col-1], cur1_cells[col], cur1_cells[col+1]);
				else
					new0_cells[col] = cur0_cells[col];			
				if (cells1_modified & (1 << (GRID_COLS32 - 1 - col)))
					CELL32(new1_cells[col],
					       cur0_cells[col-1], cur0_cells[col], cur0_cells[col+1],
					       cur1_cells[col-1], cur1_cells[col], cur1_cells[col+1],
					       bot_cells[col-1], bot_cells[col], bot_cells[col+1]);
				else
					new1_cells[col] = cur1_cells[col];
			}
#endif

		}
		if (cells01_modified & (1 << 0))
			CELL64(new0_cells[GRID_COLS32-1], new1_cells[GRID_COLS32-1],
			       top_cells[GRID_COLS32-2], top_cells[GRID_COLS32-1], 0,
			       cur0_cells[GRID_COLS32-2], cur0_cells[GRID_COLS32-1], 0,
			       cur1_cells[GRID_COLS32-2], cur1_cells[GRID_COLS32-1], 0,
			       bot_cells[GRID_COLS32-2], bot_cells[GRID_COLS32-1], 0);
		else {
			new0_cells[GRID_COLS32-1] = cur0_cells[GRID_COLS32-1];
			new1_cells[GRID_COLS32-1] = cur1_cells[GRID_COLS32-1];
		}
		for (col = 0; col < GRID_COLS32; col++) {
			line0_modified = (line0_modified << 1) | ((wa_cur0[FIFO_HEADER_WIDTH+col] != ra_cur0[FIFO_HEADER_WIDTH+col]) ? 1 : 0);
			line1_modified = (line1_modified << 1) | ((wa_cur1[FIFO_HEADER_WIDTH+col] != ra_cur1[FIFO_HEADER_WIDTH+col]) ? 1 : 0);
		}
		wa_cur0[0] = line0_modified;
		wa_cur1[0] = line1_modified;
	}
	return line0_modified | line1_modified;
}

/*****************************************************************************
 * Algorithm 0: Time pipeline + UDN
 *****************************************************************************/
#define init_state() do { prev_state = 2; state = 0; next_state = 1;} while (0)
//#define advance_state() do { prev_state = state; state = next_state; next_state = next_state_tab[state];} while (0)
#define advance_state() do { tmp_state = prev_state; prev_state = state; state = next_state; next_state = tmp_state;} while (0)

static void algorithm_time_pipeline_udn(tile_data_t *td, int num_tiles, int num_steps)
{
	unsigned int slines[4][FIFO_LINE_WIDTH];
	int row, step;
	int tmp_state, prev_state, state, next_state;

	memset(slines, 0, 4 * FIFO_LINE_WIDTH * sizeof(int));

	step = td->id;

	if (step == 0) {
		init_state();
		memcpy(slines[2], td->private_grid[0], FIFO_LINE_WIDTH * sizeof(int));
		for (row = 0; row < GRID_ROWS; row++) {
			if (row != GRID_ROWS - 1)
				memcpy(slines[state], td->private_grid[0] + (row+1) * FIFO_LINE_WIDTH, FIFO_LINE_WIDTH * sizeof(int));
			if (step != num_steps - 1)
				process_line_and_send(td, 1, slines[3], (row == 0) ? td->zeros : slines[next_state], slines[prev_state], (row == GRID_ROWS - 1) ? td->zeros : slines[state]);
			else
				process_line(1, td->private_grid[1] + row * FIFO_LINE_WIDTH, (row == 0) ? td->zeros : slines[next_state], slines[prev_state], (row == GRID_ROWS - 1) ? td->zeros : slines[state]);
			advance_state();
		}
		step += num_tiles;
		if (num_steps == 1)
			td->wrote_output_grid = 1;
	}
	for (; step < num_steps - 1; step += num_tiles) {
		init_state();

		// read 1st line
		receive_cells(td, slines[2], 0);

		row = 0;
		if (row != GRID_ROWS - 1)
			receive_cells(td, slines[state], row);
		process_line_and_send(td, 1, slines[3], td->zeros, slines[prev_state], (row == GRID_ROWS - 1) ? td->zeros : slines[state]);
		row = 1;
		advance_state();

		// main loop
		for (; row < GRID_ROWS - 1; row++) {
			// read next line
			receive_cells(td, slines[state], row);
			process_line_and_send(td, 1, slines[3], slines[next_state], slines[prev_state], slines[state]);
			advance_state();
		}
		if (row == GRID_ROWS - 1) {
			process_line_and_send(td, 1, slines[3], slines[next_state], slines[prev_state], td->zeros);
			advance_state();
		}

	}
	if (step == num_steps - 1) {
		init_state();
		receive_cells(td, slines[2], 0);
		for (row = 0; row < GRID_ROWS; row++) {
			if (row != GRID_ROWS - 1)
				receive_cells(td, slines[state], row + 1);
			process_line(1, td->private_grid[1] + row * FIFO_LINE_WIDTH, (row == 0) ? td->zeros : slines[next_state], slines[prev_state], (row == GRID_ROWS - 1) ? td->zeros : slines[state]);
			advance_state();
		}
		td->wrote_output_grid = 1;
	}
}
/*****************************************************************************
 * Algorithm 1: Time pipeline + UDN, each tile does 2 steps at a time
 *****************************************************************************/
static void algorithm_time_pipeline_2steps_udn(tile_data_t *td, int num_tiles, int num_steps)
{
	unsigned int slines[3][3][FIFO_LINE_WIDTH];
	int row, step;
	int tmp_state, prev_state, state, next_state;
	int frame_modified = 0;

	memset(slines, 0, 9 * FIFO_LINE_WIDTH * sizeof(int));

	for (step = td->id * 2; step < num_steps; step += 2 * num_tiles) {
		init_state();
		if (step == 0)
			memcpy(slines[0][2], td->private_grid[0], FIFO_LINE_WIDTH * sizeof(int));
		else
			receive_cells(td, slines[0][2], 0);
		for (row = 0; row < GRID_ROWS + 1; row++) {
			if (row < GRID_ROWS - 1) {
				if (step == 0)
					memcpy(slines[0][state], td->private_grid[0] + (row+1) * FIFO_LINE_WIDTH, FIFO_LINE_WIDTH * sizeof(int));
				else
					receive_cells(td, slines[0][state], row + 1);
			}
			if (row < GRID_ROWS)
				frame_modified |= process_line(1, (step == num_steps - 1) ? (td->private_grid[1] + row * FIFO_LINE_WIDTH) : slines[1][state], (row == 0) ? td->zeros : slines[0][next_state], slines[0][prev_state], (row == GRID_ROWS - 1) ? td->zeros : slines[0][state]);
			if (row > 0) {
				if (step == num_steps - 2)
					frame_modified |= process_line(1, td->private_grid[1] + (row - 1) * FIFO_LINE_WIDTH, (row - 1 == 0) ? td->zeros : slines[1][next_state], slines[1][prev_state], (row - 1 == GRID_ROWS - 1) ? td->zeros : slines[1][state]);
				else if (step != num_steps - 1)
					frame_modified |= process_line_and_send(td, 1, slines[2][prev_state], (row - 1 == 0) ? td->zeros : slines[1][next_state], slines[1][prev_state], (row - 1 == GRID_ROWS - 1) ? td->zeros : slines[1][state]);
			}
			advance_state();
		}
	}
	if ((step - 2  * num_tiles) >= (num_steps - 2) && (step - 2  * num_tiles) <= (num_steps - 1))
		td->wrote_output_grid = 1;
}

/*****************************************************************************
 * Algorithm 2: Time pipeline + UDN, each tile does 3 steps at a time
 *****************************************************************************/
static void algorithm_time_pipeline_3steps_udn(tile_data_t *td, int num_tiles, int num_steps)
{
	unsigned int slines[4][3][FIFO_LINE_WIDTH];
	int row, step;
	int tmp_state, prev_state, state, next_state;
	int frame_modified = 0;

	memset(slines, 0, 12 * FIFO_LINE_WIDTH * sizeof(int));

	for (step = td->id * 3; step < num_steps; step += 3 * num_tiles) {
		init_state();
		//		state = 0; prev0_state = 2; prev1_state = 1;
		if (step == 0)
			memcpy(slines[0][2], td->private_grid[0], FIFO_LINE_WIDTH * sizeof(int));
		else
			receive_cells(td, slines[0][2], 0);
		for (row = 0; row < GRID_ROWS + 2; row++) {
			if (row < GRID_ROWS - 1) {
				if (step == 0)
					memcpy(slines[0][state], td->private_grid[0] + (row+1) * FIFO_LINE_WIDTH, FIFO_LINE_WIDTH * sizeof(int));
				else
					receive_cells(td, slines[0][state], row + 1);
			}
			if (row < GRID_ROWS)
				frame_modified |= process_line(1, (step == num_steps - 1) ? (td->private_grid[1] + row * FIFO_LINE_WIDTH) : slines[1][state], (row == 0) ? td->zeros : slines[0][next_state], slines[0][prev_state], (row == GRID_ROWS - 1) ? td->zeros : slines[0][state]);
			if (row > 0 && row < GRID_ROWS + 1)
				frame_modified |= process_line(1, (step == num_steps - 2) ? (td->private_grid[1] + (row - 1) * FIFO_LINE_WIDTH) : slines[2][prev_state], (row - 1 == 0) ? td->zeros : slines[1][next_state], slines[1][prev_state], (row - 1 == GRID_ROWS - 1) ? td->zeros : slines[1][state]);
			if (row > 1) {
				if (step == num_steps - 3)
					frame_modified |= process_line(1, td->private_grid[1] + (row - 2) * FIFO_LINE_WIDTH , (row - 2 == 0) ? td->zeros : slines[2][state], slines[2][next_state], (row - 2 == GRID_ROWS - 1) ? td->zeros : slines[2][prev_state]);
				else if (step < num_steps - 3)
					frame_modified |= process_line_and_send(td, 1, slines[3][next_state], (row - 2 == 0) ? td->zeros : slines[2][state], slines[2][next_state], (row - 2 == GRID_ROWS - 1) ? td->zeros : slines[2][prev_state]);
			}
			advance_state();
		}
	}
	if ((step - 3  * num_tiles) >= (num_steps - 3) && (step - 3  * num_tiles) <= (num_steps - 1))
		td->wrote_output_grid = 1;

}

#ifndef SCC_PLATFORM
/*****************************************************************************
 * Algorithm 5: Time pipeline with local home fifos
 *****************************************************************************/
static void algorithm_time_pipeline_1line(tile_data_t *td, int num_tiles, int num_steps)
{
	int row, step;
	int ra_top, ra_cur, ra_bot, wa;
	int frame_modified = 0;
	unsigned int *input_fifo = td->fifo, *output_fifo = td->next->fifo;

	step = td->id;

	// first step, on tile 0: read from input array, write to tile 1 fifo
	if (step == 0) {
		wa = td->next_fifo_wa;
		for (row = 0; row < GRID_ROWS; row++) {
			//			printf("tile %d step %d row %d\n", td->id, step, row);
			fifo_wait(td, wa, 0, (step == num_steps - 1) ? 0 : FIFO_LINE_WIDTH);
			frame_modified |= process_line(1,
						     (step == num_steps - 1) ? (td->private_grid[1] + row * FIFO_LINE_WIDTH) : (output_fifo+wa),
						     (row == 0) ? td->zeros : (td->private_grid[0] + (row-1) * FIFO_LINE_WIDTH),
						     (td->private_grid[0] + row * FIFO_LINE_WIDTH),
						     (row == GRID_ROWS - 1) ? td->zeros : (td->private_grid[0]+(row+1)*FIFO_LINE_WIDTH));
			if (step != num_steps - 1) {
				wa = fifo_next_line(wa);
				update_next_tile_fifo_wa(td, wa);
			}
		}
		step += num_tiles;
		if (num_steps == 1)
			td->wrote_output_grid = 1;
	}

	/*
	 * most steps, each tile computes a generation, then skips num_tiles generations
	 */
	for (; step < num_steps - 1; step += num_tiles) {
		/*
		 * Set up line pointers
		 */
		ra_top = fifo_prev_line(td->fifo_ra);
		ra_cur = td->fifo_ra;
		ra_bot = fifo_next_line(td->fifo_ra);
		wa = td->next_fifo_wa;
		frame_modified = 0;
		for (row = 0; row < GRID_ROWS; row++) {
			// Wait for available data from the previous core
			fifo_wait(td, wa, ((row == 0 || row == GRID_ROWS - 1) ? 2 : 3) * FIFO_LINE_WIDTH, FIFO_LINE_WIDTH);

			// Generate the next line of the new generation
			frame_modified |= process_line(1,
						     output_fifo+wa,
						     (row == 0) ? td->zeros : (input_fifo+ra_top),
						     input_fifo+ra_cur,
						     (row == GRID_ROWS - 1) ? td->zeros : (input_fifo+ra_bot));

			// Update our read address, send a copy to previous core so it won't have to poll over the network.
			ra_top = fifo_next_line(ra_top);
			ra_cur = fifo_next_line(ra_cur);
			ra_bot = fifo_next_line(ra_bot);
			update_prev_tile_fifo_ra(td, ra_top);
			
			// Notify next tile that a new line is available.
			wa = fifo_next_line(wa);
			update_next_tile_fifo_wa(td, wa);
		}
		update_prev_tile_fifo_ra(td, ra_cur);
	}

	/*
	 * Last step (for num_steps > 1): write to output grid. Only one tile will reach here.
	 */
	if (step == num_steps - 1) {
		ra_top = fifo_prev_line(td->fifo_ra);
		ra_cur = td->fifo_ra;
		ra_bot = fifo_next_line(td->fifo_ra);
		wa = td->next_fifo_wa;
		frame_modified = 0;
		for (row = 0; row < GRID_ROWS; row++) {
			fifo_wait(td, wa, (((row == 0 || row == GRID_ROWS - 1) ? 2 : 3) * FIFO_LINE_WIDTH), 0);
			frame_modified |= process_line(1,
						       (td->private_grid[1] + row * FIFO_LINE_WIDTH),
						       (row == 0) ? td->zeros : (input_fifo+ra_top),
						       (input_fifo+ra_cur),
						       (row == GRID_ROWS - 1) ? td->zeros : (input_fifo+ra_bot));

			ra_top = fifo_next_line(ra_top);
			ra_cur = fifo_next_line(ra_cur);
			ra_bot = fifo_next_line(ra_bot);
			update_prev_tile_fifo_ra(td, ra_top);
		}
		update_prev_tile_fifo_ra(td, ra_cur);
		td->wrote_output_grid = 1;
	}
	update_prev_tile_fifo_ra(td, td->fifo_wa);
}

/*****************************************************************************
 * Algorithm 6: Time pipeline variant: 2 spatial lines at a time
 *****************************************************************************/
static void algorithm_time_pipeline_2lines(tile_data_t *td, int num_tiles, int num_steps)
{
	int row, step;
	int ra_top, ra_cur0, ra_cur1, ra_bot, wa_cur0, wa_cur1;
	int frame_modified = 0;
	unsigned int *input_fifo = td->fifo, *output_fifo = td->next->fifo;

	for (step = td->id; step < num_steps; step += num_tiles) {
		ra_top = (td->fifo_ra - FIFO_LINE_WIDTH + FIFO_SIZE) % FIFO_SIZE;
		ra_cur0 = td->fifo_ra;
		ra_cur1 = (td->fifo_ra + FIFO_LINE_WIDTH) % FIFO_SIZE;
		ra_bot = (td->fifo_ra + 2*FIFO_LINE_WIDTH) % FIFO_SIZE;
		wa_cur0 = td->next_fifo_wa;
		wa_cur1 = (wa_cur0 + FIFO_LINE_WIDTH) % FIFO_SIZE;
		frame_modified = 0;
		for (row = 0; row < GRID_ROWS; row += 2) {
			fifo_wait(td, wa_cur0, (step == 0) ? 0 : (((row == 0 || row == GRID_ROWS - 2) ? 3 : 4) * FIFO_LINE_WIDTH), (step == num_steps - 1) ? 0 : (2*FIFO_LINE_WIDTH));
			frame_modified |= process_2lines(1,
							 (step == num_steps - 1) ? (td->private_grid[1] + (row+0) * FIFO_LINE_WIDTH) : (output_fifo+wa_cur0),
							 (step == num_steps - 1) ? (td->private_grid[1] + (row+1) * FIFO_LINE_WIDTH) : (output_fifo+wa_cur1),
							 (row == 0) ? td->zeros : ((step == 0) ? (td->private_grid[0] + (row-1)*FIFO_LINE_WIDTH) : (input_fifo+ra_top)),
							 (step == 0) ? (td->private_grid[0] + row * FIFO_LINE_WIDTH) : (input_fifo+ra_cur0),
							 (step == 0) ? (td->private_grid[0] + (row+1) * FIFO_LINE_WIDTH) : (input_fifo+ra_cur1),
							 (row == GRID_ROWS - 2) ? td->zeros : ((step == 0) ? (td->private_grid[0] + (row+2) * FIFO_LINE_WIDTH) : (input_fifo+ra_bot)));

			if (step != 0) {
				ra_top = (ra_top + 2*FIFO_LINE_WIDTH) % FIFO_SIZE;
				ra_cur0 = (ra_cur0 + 2*FIFO_LINE_WIDTH) % FIFO_SIZE;
				ra_cur1 = (ra_cur1 + 2*FIFO_LINE_WIDTH) % FIFO_SIZE;
				ra_bot = (ra_bot + 2*FIFO_LINE_WIDTH) % FIFO_SIZE;
				update_prev_tile_fifo_ra(td, ra_top);
			}

			if (step != num_steps - 1) {
				wa_cur0 = (wa_cur0 + 2*FIFO_LINE_WIDTH) % FIFO_SIZE;
				update_next_tile_fifo_wa(td, wa_cur0);
				wa_cur1 = (wa_cur1 + 2*FIFO_LINE_WIDTH) % FIFO_SIZE;
			}

		}
		update_prev_tile_fifo_ra(td, ra_cur0);
	}
	if ((step - num_tiles) == (num_steps - 1))
		td->wrote_output_grid = 1;

	update_prev_tile_fifo_ra(td, td->fifo_wa);
}

/*****************************************************************************
 * Algorithm 7: Time pipeline variant: each tile does 2 generations
 *****************************************************************************/
static void algorithm_time_pipeline_2steps(tile_data_t *td, int num_tiles, int num_steps)
{
	unsigned int slines[3][FIFO_LINE_WIDTH];
	int row, step;
	int ra_top, ra_cur, ra_bot, wa;
	int frame_modified = 0;
	unsigned int *input_fifo = td->fifo, *output_fifo = td->next->fifo;
	int state, srow;

	memset(slines, 0, 3 * FIFO_LINE_WIDTH * sizeof(int));

	for (step = 2*td->id; step < num_steps; step += 2*num_tiles) {
		state = 0;
		ra_top = fifo_prev_line(td->fifo_ra);
		ra_cur = td->fifo_ra;
		ra_bot = fifo_next_line(td->fifo_ra);
		wa = td->next_fifo_wa;
		frame_modified = 0;
		srow = -1;
		for (row = 0; row < GRID_ROWS + 1; row++) {
			if (row != GRID_ROWS) {
				fifo_wait(td, wa, (step == 0) ? 0 : (((row == 0 || row == GRID_ROWS - 1) ? 2 : 3) * FIFO_LINE_WIDTH), (step == num_steps - 1) ? 0 : FIFO_LINE_WIDTH);
				frame_modified |= process_line(1,
							       (step == num_steps - 1) ? (td->private_grid[1] + row * FIFO_LINE_WIDTH) : slines[state],
							       (row == 0) ? td->zeros : ((step == 0) ? (td->private_grid[0] + (row-1)*FIFO_LINE_WIDTH) : (input_fifo+ra_top)),
							       (step == 0) ? (td->private_grid[0] + row * FIFO_LINE_WIDTH) : (input_fifo+ra_cur),
							       (row == GRID_ROWS - 1) ? td->zeros : ((step == 0) ? (td->private_grid[0] + (row+1) * FIFO_LINE_WIDTH) : (input_fifo+ra_bot)));
			}

			switch (state) {
			case 0:
				if (row != 0) {
					frame_modified |= process_line(1, (step == num_steps - 2) ? (td->private_grid[1] + (row-1) * FIFO_LINE_WIDTH) : (output_fifo+wa), (srow == 0) ? td->zeros : slines[1], slines[2], (srow == GRID_ROWS - 1) ? td->zeros : slines[0]);
				}
				state = 1;
				break;
			case 1:
				frame_modified |= process_line(1, (step == num_steps - 2) ? (td->private_grid[1] + (row-1) * FIFO_LINE_WIDTH) : (output_fifo+wa), (srow == 0) ? td->zeros : slines[2], slines[0], (srow == GRID_ROWS - 1) ? td->zeros : slines[1]);
				state = 2;
				break;
			case 2:
				frame_modified |= process_line(1, (step == num_steps - 2) ? (td->private_grid[1] + (row-1) * FIFO_LINE_WIDTH) : (output_fifo+wa), (srow == 0) ? td->zeros : slines[0], slines[1], (srow == GRID_ROWS - 1) ? td->zeros : slines[2]);
				state = 0;
				break;
			}
			if (step != 0 && row != GRID_ROWS) {
				ra_top = fifo_next_line(ra_top);
				ra_cur = fifo_next_line(ra_cur);
				ra_bot = fifo_next_line(ra_bot);
				update_prev_tile_fifo_ra(td, ra_top);
			}
			if ((step != ((num_steps >> 1) << 1)) && srow >= 0) {
				wa = fifo_next_line(wa);
				update_next_tile_fifo_wa(td, wa);
			}
			srow++;
		}
		update_prev_tile_fifo_ra(td, ra_cur);
	}
	update_prev_tile_fifo_ra(td, td->fifo_wa);

	if ((step - 2*num_tiles) == (num_steps - 1) || (step - 2*num_tiles) == (num_steps - 2))
		td->wrote_output_grid = 1;
}
#endif

/*****************************************************************************
 * Algorithm 3: coherent space division
 *****************************************************************************/
static void algorithm_space_division_coherent(tile_data_t *td, int num_tiles, int num_steps)
{
	int row, step = 0;
	int ra_top, ra_cur, ra_bot, wa;
	int start_row, nr_rows, end_row, sw, ew;
	int frame_modified = 0;
	unsigned int *src_grid, *dst_grid;
	unsigned int *input_grid, *output_grid;
	int overlap, max_overlap = param_max_overlap;

	//	struct timespec req = {0,100};

	nr_rows = ROUNDUP(GRID_ROWS,num_tiles) / num_tiles;
	start_row = td->id * nr_rows;
	end_row = MIN(start_row + nr_rows, GRID_ROWS);

	src_grid = coherent_grid[0];
	dst_grid = coherent_grid[1];

	overlap = max_overlap;
	input_grid = src_grid;
	output_grid = (overlap > 0) ? td->private_grid[0] : dst_grid;

	//	printf("tile %d, start_row %d, nr_rows %d, overlap %d\n", td->id, start_row, nr_rows, overlap);

	for (step = 0; step < num_steps; step++) {
		if (step == num_steps - 1) {
			overlap = 0;
			output_grid = dst_grid;
		}
		sw = MAX(start_row - overlap, 0);
		ew = MIN(end_row + overlap, GRID_ROWS);
		ra_top = (sw - 1) * FIFO_LINE_WIDTH;
		wa = ra_cur = sw * FIFO_LINE_WIDTH;
		ra_bot = (sw + 1) * FIFO_LINE_WIDTH;
		frame_modified = 0;

		row = sw;

#if 0
		for (; row < ew - 1; row += 2) {
			//			printf("tile %d step %d row %d\n", td->id, step, row);
			frame_modified |= process_2lines(1,
							 output_grid+wa,
							 output_grid+wa+FIFO_LINE_WIDTH,
							 (row == 0) ? coherent_zeros : (input_grid+ra_top),
							 input_grid+ra_cur,
							 input_grid+ra_cur+FIFO_LINE_WIDTH,
							 (row == GRID_ROWS - 2) ? coherent_zeros : (input_grid+ra_bot+FIFO_LINE_WIDTH));
			ra_top = ra_top + 2*FIFO_LINE_WIDTH;
			ra_cur = ra_cur + 2*FIFO_LINE_WIDTH;
			ra_bot = ra_bot + 2*FIFO_LINE_WIDTH;
			wa = wa + 2*FIFO_LINE_WIDTH;
		}
#endif

		for (; row < ew; row++) {
			//			printf("tile %d step %d row %d\n", td->id, step, row);
			frame_modified |= process_line(1,
						       output_grid+wa,
						       (row == 0) ? coherent_zeros : (input_grid+ra_top),
						       input_grid+ra_cur,
						       (row == GRID_ROWS - 1) ? coherent_zeros : (input_grid+ra_bot));
			ra_top = ra_top + FIFO_LINE_WIDTH;
			ra_cur = ra_cur + FIFO_LINE_WIDTH;
			ra_bot = ra_bot + FIFO_LINE_WIDTH;
			wa = wa + FIFO_LINE_WIDTH;
		}

		/*
		 * barrier
		 */
		if (overlap == 0) {
			//		printf("tile %d: entering barrier\n", td->id);
			if (num_tiles != 1)
				barrier_wait();
			overlap = max_overlap;

			// toggle pointers
			if (output_grid == coherent_grid[0]) {
				input_grid = coherent_grid[0];
				dst_grid = coherent_grid[1];
			} else {
				if (output_grid != coherent_grid[1]) {
					printf("ERROR\n");
					exit(1);
				}
				input_grid = coherent_grid[1];
				dst_grid = coherent_grid[0];
			}
			output_grid = (overlap > 0) ? td->private_grid[0] : dst_grid;
		} else {
			if (step == num_steps - 1) {
				printf("ERROR num_steps, overlap\n");
				exit(1);
			}

			if (output_grid == td->private_grid[0]) {
				output_grid = (overlap == 1) ? dst_grid : td->private_grid[1];
				input_grid = td->private_grid[0];
			} else {
				output_grid = (overlap == 1) ? dst_grid : td->private_grid[0];
				input_grid = td->private_grid[1];
			}
			overlap--;
		}
	}

	if (input_grid != coherent_grid[1])
		memcpy(coherent_grid[1], input_grid, GRID_ROWS * FIFO_LINE_WIDTH * sizeof(int));
	if (td->id == 0)
		td->wrote_output_grid = 1;
}

/*****************************************************************************
 * Algorithm 4: space division, udn+scc version
 *****************************************************************************/
static void algorithm_space_division_distributed_udn_mpb(tile_data_t *td, int num_tiles, int num_steps)
{
	int row, step = 0, db_idx = 1;
	int ra_top, ra_cur, ra_bot, wa;
	int start_row, nr_rows, end_row, sw, ew;
	int frame_modified = 0;
	unsigned int *input_grid, *output_grid;
	int overlap, max_overlap = param_max_overlap;

	//	struct timespec req = {0,100};

	nr_rows = ROUNDUP(GRID_ROWS,num_tiles) / num_tiles;
	num_tiles = ROUNDUP(GRID_ROWS,nr_rows) / nr_rows;
	start_row = td->id * nr_rows;
	end_row = MIN(start_row + nr_rows, GRID_ROWS);

	overlap = max_overlap;
	input_grid = td->private_grid[0];
	output_grid = td->private_grid[1];

	//	printf("tile %d, start_row %d, nr_rows %d, overlap %d\n", td->id, start_row, nr_rows, overlap);

#ifdef SCC_PLATFORM
	RCCE_flag_write(&td->mpb_flag0, RCCE_FLAG_UNSET, td->id); 
	RCCE_flag_write(&td->mpb_flag1, RCCE_FLAG_UNSET, td->id); 
	RCCE_flag_write(&td->mpb_flag0a, RCCE_FLAG_SET, td->id); 
	RCCE_flag_write(&td->mpb_flag1a, RCCE_FLAG_SET, td->id); 
#endif
	barrier_wait();

	for (step = 0; step < num_steps; step++) {
		if (step == num_steps - 1) {
			overlap = 0;
		}
		sw = MAX(start_row - overlap, 0);
		ew = MIN(end_row + overlap, GRID_ROWS);
		ra_top = (sw - 1) * FIFO_LINE_WIDTH;
		wa = ra_cur = sw * FIFO_LINE_WIDTH;
		ra_bot = (sw + 1) * FIFO_LINE_WIDTH;
		frame_modified = 0;

		row = sw;

#if 0
		for (; row < ew - 1; row += 2) {
			//			printf("tile %d step %d row %d\n", td->id, step, row);
			frame_modified |= process_2lines(1,
							 output_grid+wa,
							 output_grid+wa+FIFO_LINE_WIDTH,
							 (row == 0) ? td->zeros : (input_grid+ra_top),
							 input_grid+ra_cur,
							 (row == GRID_ROWS - 1) ? td->zeros : input_grid+ra_cur+FIFO_LINE_WIDTH,
							 (row >= GRID_ROWS - 2) ? td->zeros : (input_grid+ra_bot+FIFO_LINE_WIDTH));
			ra_top = ra_top + 2*FIFO_LINE_WIDTH;
			ra_cur = ra_cur + 2*FIFO_LINE_WIDTH;
			ra_bot = ra_bot + 2*FIFO_LINE_WIDTH;
			wa = wa + 2*FIFO_LINE_WIDTH;
		}
#endif

		for (; row < ew; row++) {
			//			printf("tile %d step %d row %d\n", td->id, step, row);
			frame_modified |= process_line(1,
						       output_grid+wa,
						       (row == 0) ? td->zeros : (input_grid+ra_top),
						       input_grid+ra_cur,
						       (row == GRID_ROWS - 1) ? td->zeros : (input_grid+ra_bot));
			ra_top = ra_top + FIFO_LINE_WIDTH;
			ra_cur = ra_cur + FIFO_LINE_WIDTH;
			ra_bot = ra_bot + FIFO_LINE_WIDTH;
			wa = wa + FIFO_LINE_WIDTH;
		}

		/*
		 * barrier
		 */
		if (overlap == 0) {
			//		printf("tile %d: entering barrier\n", td->id);
			// copy data to neighbour tiles
			if (num_tiles != 1) {
#ifdef PC_PLATFORM
				barrier_wait();
				if (td->id < (num_tiles-1))
					memcpy(td->next->private_grid[db_idx] + (end_row-1-max_overlap)*FIFO_LINE_WIDTH, output_grid + (end_row-1-max_overlap)*FIFO_LINE_WIDTH, (1+max_overlap)*FIFO_LINE_WIDTH*sizeof(int));
				if (td->id != 0 && td->id <= (num_tiles - 1))
					memcpy(td->prev->private_grid[db_idx] + (start_row)*FIFO_LINE_WIDTH, output_grid + (start_row)*FIFO_LINE_WIDTH, (1+max_overlap)*FIFO_LINE_WIDTH*sizeof(int));
				barrier_wait();
#endif
#if defined(TILERA_PLATFORM) || defined(SCC_PLATFORM)
				if (td->id < (num_tiles-1))
					send_cells_next(td, output_grid + (end_row-1-max_overlap)*FIFO_LINE_WIDTH, (1+max_overlap));
				if (td->id != 0 && td->id <= (num_tiles - 1))
					send_cells_prev(td, output_grid + (start_row)*FIFO_LINE_WIDTH, (1+max_overlap));
				// recv next
				if (td->id < (num_tiles-1))
					receive_cells_next(td, output_grid + (end_row)*FIFO_LINE_WIDTH, (1+max_overlap));
				// rec prev
				if (td->id != 0 && td->id <= (num_tiles - 1))
					receive_cells_prev(td, output_grid + (start_row-1-max_overlap)*FIFO_LINE_WIDTH, (1+max_overlap));
#endif
			}
			overlap = max_overlap;

		} else {
			overlap--;
		}
		// toggle pointers
		input_grid = td->private_grid[db_idx];
		output_grid = td->private_grid[db_idx^1];
		db_idx ^= 1;
	}

	if (db_idx != 0)
		memcpy(td->private_grid[1], td->private_grid[0], GRID_ROWS * FIFO_LINE_WIDTH * sizeof(int));
}

#ifdef TILERA_PLATFORM
/*
 * map id to tilera tile id
 */
static int tilera_snake_path(int id)
{
	int snake8[8] = {0,1,2,3,11,10,9,8};
	int snake16[16] = {0,1,2,3,4,5,6,7,15,14,13,12,11,10,9,8};
	int snake24[24] = {0,1,2,3,4,5,6,7,15,23,22,14,13,21,20,12,11,19,18,10,9,17,16,8};
	int snake32[32] = {0,1,2,3,4,5,6,7,15,23,31,30,22,14,13,21,29,28,20,12,11,19,27,26,18,10,9,17,25,24,16,8};
	int snake40[40] = {
		0,1,2,3,4,5,6,7, // right
		15,23,31,39, // down
		38,30,22,14, // up
		13,21,29,37, // down
		36,28,20,12, // up
		11,19,27,35, // down
		34,26,18,10, // up
		9, 17,25,33, // down
		32,24,16,8 };
	int snake48[48] = {
		0,1,2,3,4,5,6,7, // right
		15,23,31,39,47, // down
		46,38,30,22,14, // up
		13,21,29,37,45, // down
		44,36,28,20,12, // up
		11,19,27,35,43, // down
		42,34,26,18,10, // up
		9, 17,25,33,41, // down
		40,32,24,16,8 };
	int snake56[56] = {
		0, 1, 2, 3, 4, 5, 6, 7, // right
		15,23,31,39,47,55,   // down
		54,46,38,30,22,14,   // up
		13,21,29,37,45,53,   // down
		52,44,36,28,20,12,   // up
		11,19,27,35,43,51,   // down
		50,42,34,26,18,10,   // up
		 9,17,25,33,41,49,   // down	
		48,40,32,24,16,8 };
	int snake62[62] = {
		0, 1, 2, 3, 4, 5, 6, 7, // right
		15,23,31,39,47,55,      // down
		54,46,38,30,22,14,      // up
		13,21,29,37,45,53,61,   // down
		60,52,44,36,28,20,12,   // up
		11,19,27,35,43,51,59,   // down
		58,50,42,34,26,18,10,   // up
		 9,17,25,33,41,49,57,   // down	
		56,48,40,32,24,16,8 };
#if 1
	if (param_num_tiles <= 8)
		return snake8[id];
	else if (param_num_tiles <= 16)
		return snake16[id];
	else if (param_num_tiles <= 24)
		return snake24[id];
	else if (param_num_tiles <= 32)
		return snake32[id];
	else if (param_num_tiles <= 40)
		return snake40[id];
	else if (param_num_tiles <= 48)
		return snake48[id];
	else if (param_num_tiles <= 56)
		return snake56[id];
	else if (param_num_tiles <= 62)
		return snake62[id];
	else {
		printf("ERROR: id %d not supported for snake path\n", id);
		exit(1);
	}
#endif
#if 0
	return id;
#endif
#if 0
	if (param_num_tiles == 48) {
		return snake48[id];
	} else if (param_num_tiles <= 62) {
		if (id >= 0 && id <= 7)
			return id;
		else if (id >= 8 && id <= 15)
			return 15 - id + 8;
		else if (id >= 16 && id <= 23)
			return id;
		else if (id >= 24 && id <= 31)
			return 31 - id + 24;
		else if (id >= 32 && id <= 39)
			return id;
		else if (id >= 40 && id <= 47)
			return 47 - id + 40;
		else if (id >= 48 && id <= 55)
			return id;
		else
			return 61 - id + 56;
	}
#endif
#if 0
	  return id;
#endif
}
#endif

static tile_data_t *allocate_tile_data(int tile_idx)
{
	tile_data_t *td;

#ifdef PC_PLATFORM
	td = (tile_data_t *) valloc(sizeof(tile_data_t));
#endif
#ifdef TILERA_PLATFORM
	int tilera_tile_id = tilera_snake_path(tile_idx);

	tmc_alloc_t alloc = TMC_ALLOC_INIT;
	tmc_alloc_set_home(&alloc, tilera_tile_id);
	//	tmc_alloc_set_home(&alloc, TMC_ALLOC_HOME_HASH);
	td = (tile_data_t *) tmc_alloc_map(&alloc, sizeof(tile_data_t));
#endif
#ifdef SCC_PLATFORM
	td = (tile_data_t *) valloc(sizeof(tile_data_t));
#endif
	if (td == NULL) {
		printf("ERROR: couldn't allocate %d bytes for tile %d\n", sizeof(tile_data_t), tile_idx);
		exit(1);
	}
	memset(td, 0, sizeof(tile_data_t));

	td->id = tile_idx;
	td->next_id = ((tile_idx + 1) % param_num_tiles);
	td->prev_id = (((tile_idx - 1) + param_num_tiles) % param_num_tiles);
#ifdef TILERA_PLATFORM
	td->tilera_tile_id = tilera_tile_id;
#endif
#ifdef SCC_PLATFORM
	if (param_algorithm == ALGORITHM_SPACE_DIVISION_DISTRIBUTED_UDN_MPB) {
		td->mpb_buff = (unsigned int *) RCCE_malloc(2*FIFO_LINE_WIDTH*sizeof(int) * (param_max_overlap + 1));
		if (RCCE_flag_alloc(&td->mpb_flag0) || RCCE_flag_alloc(&td->mpb_flag1) || RCCE_flag_alloc(&td->mpb_flag0a) || RCCE_flag_alloc(&td->mpb_flag1a) || td->mpb_buff == NULL) {
			    printf("ERROR: couldn't allocate flags or %d bytes for MPB buffer\n", 2*FIFO_LINE_WIDTH*sizeof(int) * (param_max_overlap + 1));
			    exit(1);
		}
	} else {
		//	td->fifo = (volatile unsigned int *) RCCE_malloc(FIFO_SIZE * sizeof(int));
		//	td->mpb_fifo_wa = (volatile unsigned int *) RCCE_malloc(FIFO_LINE_WIDTH);
		//	td->mpb_fifo_ra = (volatile unsigned int *) RCCE_malloc(FIFO_LINE_WIDTH);
		// FIXME
		td->fifo = (unsigned int *) RCCE_malloc(FIFO_SIZE * sizeof(int));
		td->mpb_fifo_wa = (unsigned int *) RCCE_malloc(FIFO_LINE_WIDTH * sizeof(int));
		td->mpb_fifo_ra = (unsigned int *) RCCE_malloc(FIFO_LINE_WIDTH * sizeof(int));

		//	printf("tile%d: MPB: td->fifo %p, td->mpb_fifo_wa %p, td->mpb_fifo_ra %p\n", tile_idx, td->fifo, td->mpb_fifo_wa, td->mpb_fifo_ra);

		if (!td->fifo || !td->mpb_fifo_wa || !td->mpb_fifo_ra) {
			printf("ERROR: RCCE_malloc returned NULL, out of memory, tried %d, line width %d, RCCE_LINE_SIZE %d\n", FIFO_SIZE * sizeof(int), FIFO_LINE_WIDTH, RCCE_LINE_SIZE);
			exit(1);
		}
	}
#endif
	return td;
}

static void log_results(double throughput)
{
	char name[80], suffix[80];
	FILE *fp;

	// log results
#ifdef TILERA_PLATFORM
	strcpy(name, "tilera_");
#endif
#ifdef SCC_PLATFORM
	strcpy(name, "scc_");
#endif
#ifdef PC_PLATFORM
	strcpy(name, "pc_");
#endif
	if (param_algorithm == ALGORITHM_SPACE_DIVISION_COHERENT || param_algorithm == ALGORITHM_SPACE_DIVISION_DISTRIBUTED_UDN_MPB)
		sprintf(suffix, "perf%d_o%d.txt", param_algorithm, param_max_overlap);
	else
		sprintf(suffix, "perf%d.txt", param_algorithm);
	strcat(name, suffix);
	fp = fopen(name, "a+");
	if (fp) {
		fprintf(fp, "%d %d %d %d %.5f\n", param_num_tiles, param_num_steps, GRID_COLS, GRID_ROWS, throughput);
	}
	fclose(fp);
}

static void *tile_run(void *tile_param)
{
	int id;
	tile_data_t *td;
	double cc_per_cell, seconds;
	int i, nr_rows, start_row, end_row;

	id = (int) tile_param;

	barrier_wait();

#ifdef TILERA_PLATFORM
	int tilera_tile_id = tilera_snake_path(id);
	//	printf("\nthread %d starting on tile %d (%d,%d)\n", id, tilera_tile_id, tilera_tile_id % 8, tilera_tile_id / 8);
	if (tmc_cpus_set_my_cpu(tilera_tile_id) != 0)
		tmc_task_die("tmc_cpus_set_my_cpu() failed (%d). CPU=%d.\n", tmc_cpus_set_my_cpu(tilera_tile_id), tilera_tile_id);

	// Now that we're bound to a core, attach to our UDN rectangle.
	if (tmc_udn_activate() < 0)
		tmc_task_die("Failure in 'tmc_udn_activate()'.");

	//	printf("tile %d (%d,%d) UDN coordinates: %d, %d\n", id, id & 7, id / 8, udn_tile_coord_x(), udn_tile_coord_y());
#endif

	td = allocate_tile_data(id);

#if defined(TILERA_PLATFORM) || defined(PC_PLATFORM)
	tile_data_ptrs[id] = td;

	barrier_wait();

	if (id == 0) {
		for (i = 0; i < param_num_tiles; i++) {
			// connect the tile data fifos in a circle:
			// 0 ->	1 -> 2 -> ... -> (param_num_tiles - 1) -> 0
			tile_data_ptrs[i]->next = tile_data_ptrs[(i+1) % param_num_tiles];
			tile_data_ptrs[i]->prev = tile_data_ptrs[(i-1+param_num_tiles) % param_num_tiles];
		}
#ifdef TILERA_PLATFORM
		for (i = 0; i < param_num_tiles; i++) {
			tile_data_ptrs[i]->next_tilera_tile_id = tile_data_ptrs[i]->next->tilera_tile_id;
			tile_data_ptrs[i]->next_header = tmc_udn_header_from_cpu(tile_data_ptrs[i]->next_tilera_tile_id);
			tile_data_ptrs[i]->next_header_word = tile_data_ptrs[i]->next_header.word+1+GRID_COLS32+1;

			tile_data_ptrs[i]->prev_tilera_tile_id = tile_data_ptrs[i]->prev->tilera_tile_id;
			tile_data_ptrs[i]->prev_header = tmc_udn_header_from_cpu(tile_data_ptrs[i]->prev_tilera_tile_id);
			tile_data_ptrs[i]->prev_header_word = tile_data_ptrs[i]->prev_header.word+1+GRID_COLS32+1;
		}
#endif
	}
#endif

	/*
	 * read input file
	 *
	 * coherent space division: tile 0 reads to coherent shared grid.
	 * space division udn: each reads to its private grid
	 * time pipeline: tile 0 reads to its private grid copy
	 */
	if (param_algorithm == ALGORITHM_SPACE_DIVISION_COHERENT) {
		if (td->id == 0)
			read_input(param_input_file_name, coherent_grid[0]);
	} else if (param_algorithm == ALGORITHM_SPACE_DIVISION_DISTRIBUTED_UDN_MPB) {
#ifdef SCC_PLATFORM
		read_input(param_input_file_name, td->private_grid[0]);
#endif
#if defined(PC_PLATFORM) || defined(TILERA_PLATFORM)
		if ((strcmp(param_input_file_name, "random") == 0) && param_num_tiles > 1) {
			if (td->id == 0)
				read_input(param_input_file_name, coherent_grid[0]);
			barrier_wait();
			memcpy(td->private_grid[0], coherent_grid[0], GRID_ROWS * FIFO_LINE_WIDTH * sizeof(int));
		} else
			read_input(param_input_file_name, td->private_grid[0]);
#endif
	} else {
		if (td->id == 0)
			read_input(param_input_file_name, td->private_grid[0]);
	}

	barrier_wait();
	//=========================================================================
	// START counting cycles at thread 0, after all reached the barrier
	//=========================================================================
	if (td->id == 0)
		stopwatch_start();

	if (td->id < param_num_tiles) {
		switch (param_algorithm) {
		case ALGORITHM_TIME_PIPELINE_UDN:
			algorithm_time_pipeline_udn(td, param_num_tiles, param_num_steps);
			break;

		case ALGORITHM_TIME_PIPELINE_2STEPS_UDN:
			algorithm_time_pipeline_2steps_udn(td, param_num_tiles, param_num_steps);
			break;

		case ALGORITHM_TIME_PIPELINE_3STEPS_UDN:
			algorithm_time_pipeline_3steps_udn(td, param_num_tiles, param_num_steps);
			break;

		case ALGORITHM_SPACE_DIVISION_COHERENT:
			algorithm_space_division_coherent(td, param_num_tiles, param_num_steps);
			break;

		case ALGORITHM_SPACE_DIVISION_DISTRIBUTED_UDN_MPB:
			algorithm_space_division_distributed_udn_mpb(td, param_num_tiles, param_num_steps);
			break;

#ifndef SCC_PLATFORM
		case ALGORITHM_TIME_PIPELINE_MEMFIFO:
			algorithm_time_pipeline_1line(td, param_num_tiles, param_num_steps);
			break;

		case ALGORITHM_TIME_PIPELINE_2LINES_MEMFIFO:
			algorithm_time_pipeline_2lines(td, param_num_tiles, param_num_steps);
			break;

		case ALGORITHM_TIME_PIPELINE_2STEPS_MEMFIFO:
			algorithm_time_pipeline_2steps(td, param_num_tiles, param_num_steps);
			break;
#endif

		default:
			printf("ERROR: unsupported algorithm %d\n", param_algorithm);
			break;
		}
	}
	barrier_wait();

	//=========================================================================
	// STOP counting cycles at thread 0, after all reached the barrier
	//=========================================================================
	if (td->id == 0) {
		stopwatch_stop();

#ifdef TILERA_PLATFORM
		uint64_t overall_cc;
		overall_cc = stop_cc - start_cc;
		printf("done %d generations of %dx%d grid on %d tiles, total %lld cells\n", param_num_steps, GRID_COLS, GRID_ROWS, param_num_tiles, (uint64_t) param_num_steps * GRID_ROWS * GRID_COLS);
		cc_per_cell = (double) overall_cc / GRID_ROWS / GRID_COLS / param_num_steps;
		seconds = (double) overall_cc /  864583333.0;
#endif
#if defined(PC_PLATFORM) || defined(SCC_PLATFORM)
		long long overall_cc, elapsed;
		elapsed = (time_end.tv_sec * 1000000LL) + time_end.tv_usec - (time_start.tv_sec * 1000000LL) - time_start.tv_usec;
		seconds = (double) elapsed / 1000000.0;
#ifdef PC_PLATFORM
		overall_cc = (long long) (seconds * 3E9);
#endif
#ifdef SCC_PLATFORM
		overall_cc = (long long) (seconds * 0.8E9);
#endif

		cc_per_cell = (double) overall_cc / GRID_ROWS / GRID_COLS / param_num_steps;
#endif
		double throughput;

		//		throughput = 864583333.0 / cc_per_cell / 1E9;
		throughput = (double) param_num_steps * GRID_ROWS * GRID_COLS / seconds / 1E9;

		printf ("Clock Cycles elapsed: %lld (%.2f seconds, %.3f clocks/cell, %.2f cells/clock, %.2f clocks/generation, %.2f generations/sec, %.2f Gigacells/sec) \n\n", overall_cc, seconds, cc_per_cell, 1.0 / cc_per_cell, (double) overall_cc / param_num_steps, (double) param_num_steps / seconds, throughput);
		
		log_results(throughput);
	}

	/*
	 * write output
	 */
	if (param_algorithm == ALGORITHM_SPACE_DIVISION_COHERENT) {
		if (td->id == 0)
			write_output(param_output_file_name, coherent_grid[1]);
	} else if (param_algorithm == ALGORITHM_SPACE_DIVISION_DISTRIBUTED_UDN_MPB) {
		barrier_wait();
		if (td->id == 0) {
			FILE *fp = fopen(param_output_file_name, "w");
			if (fp == NULL) {
				printf("ERROR: couldn't open output file %s\n", param_output_file_name);
				exit(1);
			}
			fclose(fp);
		}
		barrier_wait();
		nr_rows = ROUNDUP(GRID_ROWS, param_num_tiles) / param_num_tiles;
		for (i = 0; i < param_num_tiles; i++) {
			barrier_wait();
			if (td->id == i) {
				start_row = td->id * nr_rows;
				end_row = MIN(start_row + nr_rows, GRID_ROWS);
				if (start_row <= GRID_ROWS - 1)
					write_output_chunk(param_output_file_name, td->private_grid[1], start_row, end_row);
			}
		}
	} else {
		if (td->wrote_output_grid)
			write_output(param_output_file_name, td->private_grid[1]);
	}

	//	printf("\ntile %d exiting\n", td->id);
	return NULL;
}

/*
 * Parse and check user arguments
 */
void parse_args(int argc, char *argv[])
{
	int display = 1;

	if (argc >= 2)
		param_num_tiles = atoi(argv[1]);
	if (argc >= 3)
		param_num_steps = atoi(argv[2]);
	if (argc >= 4)
		param_input_file_name = argv[3];
	if (argc >= 5)
		param_output_file_name = argv[4];
	if (argc >= 6)
		param_algorithm = atoi(argv[5]);
	if (argc >= 7)
		param_max_overlap = atoi(argv[6]);

#ifdef SCC_PLATFORM
	if (param_num_tiles != RCCE_num_ues()) {
		printf("ERROR: num_tiles %d != RCCE_num_ues()=%d, overriding\n", param_num_tiles, RCCE_num_ues());
		param_num_tiles = RCCE_num_ues();
	}
#endif


#ifdef SCC_PLATFORM
	display = (RCCE_ue() == 0);
#endif

	if (display) {
		printf("\nanalyzing configured parameters:\n");
		printf("\tinput file: %s\n", param_input_file_name);
		printf("\toutput file: %s\n", param_output_file_name);
		printf("\talgorithm: %d\n", param_algorithm);
		printf("\tmax overlap (space division): %d\n", param_max_overlap);
		printf("\tnumber of steps: %d\n", param_num_steps);
		printf("\tnumber of tiles: %d\n", param_num_tiles);
	}

	if (param_num_tiles < 0 || param_num_tiles > MAX_TILES) {
		printf("ERROR: param_num_tiles %d out of range\n", param_num_tiles);
		exit(1);
	}
	if (GRID_COLS > 1024) {
		printf("ERROR: GRID_COLS > 1024 not supported\n");
		exit(1);
	}

#ifdef TILERA_PLATFORM
	int available_udn_lines;

	// verify that there is enough space
	if (param_algorithm == ALGORITHM_TIME_PIPELINE_UDN) {
		available_udn_lines = param_num_tiles * (3+UDN_DEMUX_WORDS / (FIFO_LINE_WIDTH - (FIFO_HEADER_WIDTH-1)));
		if (available_udn_lines < GRID_ROWS) {
			printf("algorithm %d not supported for %d tiles, available_udn_lines %d < %d grid rows\n", ALGORITHM_TIME_PIPELINE_UDN, param_num_tiles, available_udn_lines, GRID_ROWS);
			exit(1);
		}
	}
	if (param_algorithm == ALGORITHM_TIME_PIPELINE_2STEPS_UDN) {
		available_udn_lines = param_num_tiles * (4+UDN_DEMUX_WORDS / (FIFO_LINE_WIDTH - (FIFO_HEADER_WIDTH-1)));
		if (available_udn_lines < GRID_ROWS) {
			printf("algorithm %d not supported for %d tiles, available_udn_lines %d < %d grid rows\n", ALGORITHM_TIME_PIPELINE_2STEPS_UDN, param_num_tiles, available_udn_lines, GRID_ROWS);
			exit(1);
		}
	}
	if (param_algorithm == ALGORITHM_TIME_PIPELINE_3STEPS_UDN) {
		available_udn_lines = param_num_tiles * (5+UDN_DEMUX_WORDS / (FIFO_LINE_WIDTH - (FIFO_HEADER_WIDTH-1)));
		if (available_udn_lines < GRID_ROWS) {
			printf("algorithm %d not supported for %d tiles, available_udn_lines %d < %d grid rows\n", ALGORITHM_TIME_PIPELINE_3STEPS_UDN, param_num_tiles, available_udn_lines, GRID_ROWS);
			exit(1);
		}
	}
#endif
#ifdef SCC_PLATFORM
	if (param_num_tiles > 48) {
		printf("ERROR: number of tiles too big (%d)\n", param_num_tiles);
		exit(1);
	}
	if (param_algorithm >= ALGORITHM_TIME_PIPELINE_MEMFIFO) {
		printf("ERROR: memory fifo based algorithm %d not supported on SCC\n", param_algorithm);
		exit(1);
	}
	if ((param_algorithm == ALGORITHM_TIME_PIPELINE_UDN || param_algorithm == ALGORITHM_TIME_PIPELINE_2STEPS_UDN || param_algorithm == ALGORITHM_TIME_PIPELINE_3STEPS_UDN) && param_num_tiles == 1) {
		printf("algorithm %d not supported for single tile\n");
		exit(1);
	}
#endif
	int nr_rows = ROUNDUP(GRID_ROWS,param_num_tiles) / param_num_tiles;
	if (param_algorithm == ALGORITHM_SPACE_DIVISION_DISTRIBUTED_UDN_MPB) {
		if (param_max_overlap > nr_rows - 1) {
			printf("ERROR: max_overlap %d too big for this amount of processors (%d), nr_rows %d\n", param_max_overlap, param_num_tiles, nr_rows);
			exit(1);
		}
	}
}

static void init(int argc, char *argv[])
{
	int display = 1;

#ifdef TILERA_PLATFORM
	printf("Game Of Life on Tilera, copyright (C) Gadi Oxman 2012\n");
#endif
#ifdef PC_PLATFORM
	printf("Game Of Life on PC, copyright (C) Gadi Oxman 2012\n");
#endif
#ifdef SCC_PLATFORM
	display = (RCCE_ue() == 0);
	if (display)
		printf("Game Of Life on Intel SCC, copyright (C) Gadi Oxman 2012\n");
#endif

	if (display) {
		printf("usage: life [num_tiles] [num_steps] [input.txt | random] [output.txt] [algorithm] [max_overlap]\n");
		printf("- use random as input file name for random grid\n");
		printf("- algorithm:\n");

		printf("\t 0: time pipeline + udn/mpb, 1 line, 1 step (default)\n");
		printf("\t 1: time pipeline + udn/mpb, 1 line, 2 steps\n");
		printf("\t 2: time pipeline + udn/mpb, 1 line, 3 steps\n");
#if defined(TILERA_PLATFORM) || defined(PC_PLATFORM)
		printf("\t 3: space division, coherent\n");
#endif
		printf("\t 4: space division, distributed + udn/mpb\n");
#if defined(TILERA_PLATFORM) || defined(PC_PLATFORM)
		printf("\t 5: time pipeline, mem fifo, 1 line, 1 step\n");
		printf("\t 6: time pipeline, mem fifo, 2 lines, 1 step\n");
		printf("\t 7: time pipeline, mem fifo, 1 line, 2 steps\n");
#endif
		printf("- max_overlap: parameter for space division algorithm\n");
		printf("\ninitializing\n");
	}

	parse_args(argc, argv);

#ifdef TILERA_PLATFORM
	int avail, mapping_error = 0, i;
	cpu_set_t desired_cpus;

	if (tmc_cpus_get_my_affinity(&desired_cpus) != 0)
		tmc_task_die("tmc_cpus_get_my_affinity() failed.");
	avail = tmc_cpus_count(&desired_cpus);

	if (avail < param_num_tiles)
		tmc_task_die("Need %d cpus, but only %d specified in affinity!", param_num_tiles, avail);
	else
		printf ("Available CPUs = %d\n", avail);

	for (i = 0; i < param_num_tiles; i++) {
		if (i != tmc_cpus_find_nth_cpu(&desired_cpus,i)) {
			printf("mapping error: tile %d to %d\n", i, tmc_cpus_find_nth_cpu(&desired_cpus,i));
			mapping_error = 1;
		}
	}
	if (mapping_error)
		tmc_task_die("dataplane grid configuration incorrect, causing mapping error, must have 8 horizontal processors in the grid");

	if (avail > param_num_tiles) {
		/*
		 * move this waiting process outside the working grid, if another cpu is available, so
		 * it won't disturb us.
		 */
		if (tmc_cpus_set_my_cpu(param_num_tiles) != 0)
			tmc_task_die("tmc_cpus_set_my_cpu() failed (%d). CPU=%d.\n", tmc_cpus_set_my_cpu(param_num_tiles), param_num_tiles);
	}

	// Reserve the UDN rectangle that surrounds our cpus.
	if (tmc_udn_init(&desired_cpus) < 0)
		tmc_task_die("Failure in 'tmc_udn_init(0)'.");

	tmc_spin_barrier_init(&coherent_barrier, param_num_tiles);
#endif
#ifdef PC_PLATFORM
	pthread_barrier_init(&coherent_barrier, NULL, param_num_tiles);
#endif
#ifdef SCC_PLATFORM
	if (param_num_tiles != RCCE_num_ues()) {
		printf("ERROR: num_tiles %d != RCCE_num_ues()=%d\n", param_num_tiles, RCCE_num_ues());
		exit(1);
	}
#endif
	memset(coherent_zeros, 0, FIFO_LINE_WIDTH * sizeof(int));
}

static void launch_threads(void)
{
#if defined(TILERA_PLATFORM) || defined(PC_PLATFORM)
	pthread_t       threads[MAX_TILES];
	pthread_attr_t  threads_attr[MAX_TILES];
	int i, rc;

	// set default thread attributes
	for (i = 0; i < param_num_tiles; i++) {
		pthread_attr_init(&threads_attr[i]);
	}

	printf("\nrunning simulation on %d tiles\n", param_num_tiles);
	//	printf("FIFO: HEADER %d, LINE %d, TOTAL %d\n", FIFO_HEADER_WIDTH, FIFO_LINE_WIDTH, FIFO_SIZE);

	for (i = 0; i < param_num_tiles; i++) {
		//	for (i = param_num_tiles - 1; i >= 0; i--) {
		rc = pthread_create(&threads[i], &threads_attr[i], tile_run, (void *) i);
		if (rc != 0) {
			printf("ERROR: failed to launch thread %d\n", i);
		}

	}

	for (i = 0; i < param_num_tiles; i++)
		pthread_join(threads[i], NULL);
#endif
#ifdef SCC_PLATFORM
	tile_run((void *) RCCE_ue());
#endif
	//	printf("\nfinished game of life simulation\n");
}

#ifdef SCC_PLATFORM
int RCCE_APP(int argc, char **argv)
#else
int main(int argc, char *argv[])
#endif
{
#ifdef SCC_PLATFORM
	RCCE_init(&argc, &argv);
#endif
	init(argc, argv);
	launch_threads();
#ifdef SCC_PLATFORM
	RCCE_finalize();
#endif
	return 0;
}
