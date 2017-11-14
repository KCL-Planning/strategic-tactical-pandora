include Makefile.linux
#include Makefile.osx
#CCC = clang++

#ILOGDIR = /usr/local/ilog/CPLEX_Studio127

CONCERTDIR = $(ILOGDIR)/concert
CPLEXDIR = $(ILOGDIR)/cplex
CPODIR = $(ILOGDIR)/cpoptimizer

CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(ARCH)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(ARCH)
CPOLIBDIR = $(CPODIR)/lib/$(ARCH)

PORT_FLAGS = -DONE_TO_ONE_RECHARGES -DRECHARGE_LB 
#-DINSERT_SOLUTION
#DEBUG = -g
DEBUG = -O -DNDEBUG

CFLAGS = -DIL_STD $(DEBUG) $(PORT_FLAGS) -I$(CPLEXDIR)/include -I$(CPODIR)/include -I$(CONCERTDIR)/include -fPIC -fstrict-aliasing -pedantic -Wall -fexceptions -Wno-long-long -m64 -DILOUSEMT -D_REENTRANT -DILM_REENTRANT -std=c++11

LDFLAGS = -L$(CONCERTLIBDIR) -L$(CPLEXLIBDIR) -L$(CPOLIBDIR) -lcp -lconcert -lilocplex -lcplex -lm -lpthread 

#=============== SOURCES =========================
OBJ_SRCS = problem.cpp solver.cpp main.cpp

#TEST_SRCS =

OBJS = $(OBJ_SRCS:.cpp=.o)
#TEST_OBJS= $(TEST_SRCS:.cpp=.o)

#------------------------------------------------------------
#.SUFFIXES: .o .h .cpp

# ------------- Stuff you shouldn't have to change ------------------

main_bug.o: main_bug.cpp
	$(CCC) -c $(CFLAGS) -o $*.o $<

main.o: main.cpp
	$(CCC) -c $(CFLAGS) -o $*.o $<

%.o: %.cpp %.h
	$(CCC) -c $(CFLAGS) -o $*.o $<

#%.o: %.h
#	$(CCC) -c $(CFLAGS) -o $*.o $<

#---------------------------------------------------------
# FILES
#---------------------------------------------------------

cp_vrp: $(OBJS) 
	$(CCC) $(CFLAGS) $(OBJS) -o cp_vrp $(LDFLAGS)

bug_vrp: solver_bug.o 
	$(CCC) $(CFLAGS)  solver_bug.o -o bug_vrp $(LDFLAGS)

clean:
	/bin/rm -rf *.o
	/bin/rm -rf cp_vrp
	/bin/rm -rf bug_vrp

# main: 
# 	$(CCC) -o main $(CFLAGS) main.cpp problem.cpp solver.cpp $(LDFLAGS)

