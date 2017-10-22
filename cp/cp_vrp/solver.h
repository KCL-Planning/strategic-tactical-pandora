
#ifndef _SOLVER_H_
#define _SOLVER_H_

#include <vector>
#include <string>
#include <ilcp/cp.h>
#include "problem.h"

ILOSTLBEGIN

// Class for creating a model and solving the problem
class Solver {

private:
  Problem _problem;
  IloEnv _env;
  IloModel _model;

  clock_t _startTime;

  IloInt _horizon;
  IloArray<IloIntervalVarArray> _activities;
  IloIntervalVarArray _missions;
  IloIntervalVarArray _recharges;

  IloIntervalSequenceVar _vehicleSequence;
  IloIntervalVar _endOfAll; // the dummy activity that is after everything

  IloTransitionDistance _travelTimes;
  IloArray<IloIntArray> _inverseDeltaCharge; 
  // 2D array where [i][j] is the backward distance (i.e., *from* j to i)

  IloCumulFunctionExpr _vehicleEnergy; // energy level

  IloObjective _obj;

  void calculateHorizonBound(IloInt);
  void createDummyEnd();
  IloIntervalVar createDummyStart(Vehicle*);
  IloIntervalVar createActivityForMission(Vehicle*,Mission*);
  IloIntervalVar createDockActivity(IloInt);
  IloIntervalVar createRechargeActivity(IloInt);
  IloIntervalVar createUndockActivity(IloInt);
  string getRechargeWaypoint();
  IloInt getRechargeTime();

  IloInt populateTransitions();

  void printSolution(const IloCP&, int, bool = true, bool = false);

  void addRechargeBound();

public:
  Solver(IloEnv e, Problem p):
    _problem(p), _env(e), _startTime(0), 
    _horizon(IloIntMax/4), _activities(_env),
    _missions(_env), _recharges(_env),
    _endOfAll(_env), _vehicleEnergy(_env)
  {}
  ~Solver() {}

  bool createModel();
  void setObjective();
  void solve();
  void simpleSolve();


};

#endif
