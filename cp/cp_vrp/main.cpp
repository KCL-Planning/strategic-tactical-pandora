/* ------------------------------------------------------------
 CP Code for MRTA Retirement Home Problem (Kyle model)
 Purpose: Problem class and terminal arguments
 ------------------------------------------------------------ */

#include <iostream>
#include <string>
#include <vector>
#include <ilcplex/ilocplex.h>
#include <ilcp/cp.h>
#include "problem.h"
#include "solver.h"

using namespace std;

int main(int argc, const char* argv[])
{
  // Terminal example: "./main 0 2 5 100 ab 0"
  // Notes: 0 and 1 are identical seed, use "none" string for no search phases

  if (argc != 4) {
    cout << "Usage: " << argv[0] << " <instance file> <time-limit> <objective>" << endl;
    return -1;
  }

  string probFile = argv[1];
  double timeLimit = atof(argv[2]); // time limit
  int objective = atoi(argv[3]);

  Problem prob(probFile, timeLimit, objective); 
  if (!prob.readInstanceData())
    return -1;

  //prob.dumpData();

  IloEnv env;
  try {
    //cout << "Create solver and solve problem." << endl;
    Solver solver(env, prob);
    if (!solver.createModel())
      return -1;

    solver.setObjective();
    solver.simpleSolve();
  }
  catch (IloException& e) {
    env.out() << " ERROR: " << e << std::endl;
  }
  env.end();

  return 0;
}

