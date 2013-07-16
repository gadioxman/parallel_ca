parallel_ca
===========

Parallel cellular automata simulation

Game of life for PC, Tilera and Intel SCC
by Gadi Oxman

1. Makefile configuration:
--------------------------

Copy or link one of the following files to the Makefile depending on the
platform:

For PC: ln -s Makefile.pc Makefile
For Tilera: ln -s Makefile.tilera Makefile
For SCC: ln -s Makefile.scc Makefile

2. Building:
--------------------------

On the PC and Tilera, type make.
On the SCC, the directory should be under rcce/apps/life, compile using:

	make API=gory life

3. List of command line parameters:
-----------------------------------

usage: life [num_tiles] [num_steps] [input.txt | random] [output.txt] [algorithm] [max_overlap]
- use random as input file name for random grid
- algorithm:
	 0: time pipeline + udn/mpb, 1 line, 1 step (default)
	 1: time pipeline + udn/mpb, 1 line, 2 steps
	 2: time pipeline + udn/mpb, 1 line, 3 steps
	 3: space division, coherent
	 4: space division, distributed + udn/mpb
	 5: time pipeline, mem fifo, 1 line, 1 step
	 6: time pipeline, mem fifo, 2 lines, 1 step
	 7: time pipeline, mem fifo, 1 line, 2 steps
- max_overlap: parameter for space division algorithm

4. Running on the PC:
---------------------

	Run the life executable followed by command line args, for example:

		./life 2 10000 random output.txt 4

	This will run on 2 processors, for 10000 generations, with random
input, writing the output to the file output.txt, with algorithm #4. If you
want to use an input file, give its name instead of "random".

5. Running on Tilera:
---------------------

	Use the run_life.sh with the same parameters, for example:

	make; ./run_life.sh 48 10000 random output.txt 0

6. Running on the Intel SCC:
----------------------------

	First copy the executable to your directory on /shared.
	Then run using rccerun, for example:

	./rccerun -nue 8 -f allhosts -clock 0.800 life 1 10000 random /shared/gdaliaox/bin_stress/output.txt 2

7. The grid size is fixed in the defines GRID_ROWS and GRID_COLS for compiler optimizations, you have to recompile if you want to change it.
