/* ------------------------------------------------------------
 CP Code for MRTA Retirement Home Problem (Kyle model)
 Purpose: Primary functions
 ------------------------------------------------------------ */

#include "solver.h"
#include <math.h>
#include <ilcp/cp.h>
#include <ilcplex/ilocplex.h>
#include <algorithm>



ILOSTLBEGIN

#define DISCRETE_MULTIPLIER 10000
//#define DISCRETE_MULTIPLIER 1

IloInt Discretize(float f) { return (IloInt) (f * DISCRETE_MULTIPLIER); }
float Undiscretize(IloInt i) { return ((float) i) / DISCRETE_MULTIPLIER; }

void Solver::calculateHorizonBound(IloInt maxTransition) {

#ifndef NDEBUG
  cout << "MaxTransition: " << maxTransition << endl;
#endif
   int rechargeIndex = _problem.getWaypointIndex()[getRechargeWaypoint()];
   for(int i = 0; i < _problem.getWaypointIndex().size(); ++i) {
     IloInt d = Discretize((_problem.getDistance(i, rechargeIndex) + 
			    _problem.getDistance(rechargeIndex, i)) * 
			   _problem.getTravelDurationCoeff());
     if (maxTransition < d) 
       maxTransition = d;
   }

   IloInt missionTime = 0;
   for(Mission *m : _problem.getMissions()) 
     missionTime += Discretize(m->getDuration());

   IloInt totalRechargeTime = Discretize(getRechargeTime() * _problem.getMissions().size());

   IloInt totalTravelTime = maxTransition * _problem.getMissions().size();

   // if we do all missions, require the maximum travel between each mission, and do 
   // a recharge before each mission
   _horizon = totalRechargeTime + totalTravelTime + missionTime;

#ifndef NDEBUG
   cout << "Horizon: " << Undiscretize(_horizon) << " == " 
	<< totalRechargeTime << " (recharge) + " << totalTravelTime << " (travel) + "
	<< missionTime << " (mission)"
	<< "\nMaxTransition: " << maxTransition << endl;
#endif
}

void Solver::createDummyEnd() {
  // initialize dummy activity that will be after all other activities
  _endOfAll.setName("Dummy End");
  _endOfAll.setLengthMin(0);
  _endOfAll.setLengthMax(0);

  _endOfAll.setEndMax(_horizon);
}

IloInt Solver::populateTransitions() {
  map<string, int>& waypointIndex = _problem.getWaypointIndex();

  // distance matrix is for all pairs of waypoints plus one for the dummy end activity
  // ctor initializes all distances to 0 so we only need to add differences
  _travelTimes = IloTransitionDistance(_env, waypointIndex.size() + 1);

  // second representation of same distance data for use in the element constraint
  _inverseDeltaCharge = IloArray<IloIntArray>(_env);
  for(int i = 0; i < waypointIndex.size() + 1; ++i) {
    _inverseDeltaCharge.add(IloIntArray(_env));
    for(int j = 0; j <  waypointIndex.size() + 1; ++j) {
      _inverseDeltaCharge[i].add(0);
    }
  }

  IloInt maxTransition = 0;

  for(int i = 0; i < waypointIndex.size(); ++i) {
    for(int j = 0; j < waypointIndex.size(); ++j) {
      // even though we have one more type, all distances to the last type are zero
      // and so the above loops do not need to iterate over the last type
      IloInt d = Discretize(_problem.getDistance(i,j) * _problem.getTravelDurationCoeff());
      _travelTimes.setValue(i,j, d);
      _inverseDeltaCharge[j][i] = Discretize(_problem.getDistance(i,j) * 
					     _problem.getTravelChargeCoeff()); 
      // reversed indices as we want _inverseDeltaCharge[x] 
      // to be the charge to go from each type to type x 
 
      if (d > maxTransition)
	maxTransition = d;
    }
  }

#ifndef NDEBUG
  for(int i = 0; i < waypointIndex.size(); ++i) {
    for(int j = 0; j < waypointIndex.size(); ++j) {

      cout << Undiscretize(_inverseDeltaCharge[i][j]) << " ";
    }
    cout << endl;
  }
#endif

  return maxTransition;
}

IloIntervalVar Solver::createDummyStart(Vehicle *v) {
  IloIntervalVar start(_env);

  start.setLengthMin(0);
  start.setLengthMax(0);
  start.setStartMin(0);
  start.setStartMax(0);
  start.setName(string(v->getName() + "_" + "Dummy Start").c_str());

  return start;
}

IloIntervalVar Solver::createActivityForMission(Vehicle *v, Mission *m) {

  IloIntervalVar act(_env);
  act.setName(string(v->getName() + "_" + m->getName()).c_str());
  act.setObject(m);
  
  IloInt duration = Discretize(m->getDuration());
  act.setLengthMin(duration);
  act.setLengthMax(duration);

  // All missions are optional and we want to maximize the ones we can achieve
  act.setOptional();
      
  // deadline
  IloNum deadline = m->getDeadline();
  if (deadline != -1) 
    act.setEndMax(Discretize(deadline));

  return act;
}

string Solver::getRechargeWaypoint() {
  vector<Recharge*> recharges = _problem.getRecharges();
  assert(recharges.size() == 1); // only one recharge location
  Recharge *r = recharges[0];
  return r->getWaypoint();
}

IloInt Solver::getRechargeTime() {
  vector<Recharge*> recharges = _problem.getRecharges();
  assert(recharges.size() == 1); // only one recharge location
  Recharge *r = recharges[0];

  return Discretize(r->getDockDuration() + r->getDuration() + r->getUndockDuration());
}

IloIntervalVar Solver::createDockActivity(IloInt i) {
  vector<Recharge*> recharges = _problem.getRecharges();
  assert(recharges.size() == 1); // only one recharge location
  Recharge *r = recharges[0];
  //cout << *r << endl;

  IloIntervalVar dockAct(_env);

  IloInt dur = Discretize(r->getDockDuration());

  dockAct.setLengthMin(dur);
  dockAct.setLengthMax(dur);
  dockAct.setOptional();

  dockAct.setName(string("dock" + to_string(i)).c_str());
  dockAct.setObject(r);

  //cout << r->getWaypoint() << endl;

  return dockAct;
}

IloIntervalVar Solver::createRechargeActivity(IloInt i) {
  vector<Recharge*> recharges = _problem.getRecharges();
  assert(recharges.size() == 1); // only one recharge location
  Recharge *r = recharges[0];
  //cout << *r << endl;

  IloIntervalVar rechargeAct(_env);

  IloInt dur = Discretize(r->getDuration());

  rechargeAct.setLengthMin(dur);
  rechargeAct.setLengthMax(dur);
  rechargeAct.setOptional();

  rechargeAct.setName(string("recharge" + to_string(i)).c_str());
  rechargeAct.setObject(r);

  //cout << r->getWaypoint() << endl;

  return rechargeAct;
}

IloIntervalVar Solver::createUndockActivity(IloInt i) {
  vector<Recharge*> recharges = _problem.getRecharges();
  assert(recharges.size() == 1); // only one recharge location
  Recharge *r = recharges[0];
  //cout << *r << endl;

  IloIntervalVar undockAct(_env);

  IloInt dur = Discretize(r->getUndockDuration());

  undockAct.setLengthMin(dur);
  undockAct.setLengthMax(dur);
  undockAct.setOptional();

  undockAct.setName(string("undock" + to_string(i)).c_str());
  undockAct.setObject(r);

  //cout << r->getWaypoint() << endl;

  return undockAct;
}

void Solver::addRechargeBound() {
 
  vector<IloInt> missionCharge;

  // set the energy consumption of the activity
  for(int i = 0; i < _missions.getSize(); ++i) {
    Mission *m = (Mission *) _missions[i].getObject();
    missionCharge.push_back(Discretize(m->getCharge()));
  }

  sort(missionCharge.begin(), missionCharge.end());
  for(int i = 0; i < missionCharge.size(); ++i)
    cout << Undiscretize(missionCharge[i]) << " ";
  cout << endl;

  // create LB on charge expended on missions if i is the number of missions
  IloIntArray chargeLB(_env);
  chargeLB.add(0);
  chargeLB.add(missionCharge[0]);
  
  for(int i = 1; i < missionCharge.size(); ++i) {
    missionCharge[i] += missionCharge[i-1];
    cout << Undiscretize(missionCharge[i]) << " ";
    chargeLB.add(missionCharge[i]);
  }
  cout << endl;

  const vector<Vehicle*> vehicles = _problem.getVehicles();
  assert(vehicles.size() == 1);
  Vehicle *v = vehicles[0];
  IloInt startCharge = Discretize(v->getStartCharge());
  cout << "startCharge: " << Undiscretize(startCharge) << endl;

  // find minimum energy to get a recharge
  int rechargeIndex = _problem.getWaypointIndex()[getRechargeWaypoint()];
  cout << "rechargeIndex: " << rechargeIndex << endl;
  int nbWaypoints = _inverseDeltaCharge[rechargeIndex].getSize() - 1; 
  // last index is end point which we can ignore for these purposes
  IloInt minDelta = IloIntMax;
  for(int i = 0; i < nbWaypoints; ++i) {
    if (i != rechargeIndex) {
      IloInt roundTrip = _inverseDeltaCharge[rechargeIndex][i] + 
	_inverseDeltaCharge[i][rechargeIndex];
      if (roundTrip < minDelta) 
	minDelta = roundTrip;
    }
  }

  assert(_recharges.getSize() > 0);
  Recharge *r = (Recharge *) _recharges[0].getObject();

  // energy consumption by dockAct
  minDelta = Discretize(r->getEndCharge()) - (minDelta + Discretize(r->getStartCharge()));

  
  cout << "minDelta: " << Undiscretize(minDelta) << endl;

  IloIntVar nbMissions(_env);
  IloIntExprArray doMission(_env);
  for(int i = 0; i < _missions.getSize(); ++i) 
    doMission.add(IloPresenceOf(_env, _missions[i]));
  _model.add(nbMissions == IloSum(doMission));

  IloIntVar missionEnergy(_env);
  _model.add(missionEnergy == chargeLB[nbMissions]);

  IloIntVar nbRecharges(_env);
  IloIntExprArray doRecharge(_env);
  for(int i = 0; i < _recharges.getSize(); ++i) 
    doRecharge.add(IloPresenceOf(_env, _recharges[i]));
  _model.add(nbRecharges == IloSum(doRecharge));

  _model.add(nbRecharges >= (missionEnergy - startCharge) / minDelta);

}

bool Solver::createModel() {

  _startTime = clock();

  _model = IloModel(_env);

  // maxTransition is the maximum time required to go between any pair of waypoints
  IloInt maxTransition = populateTransitions();
  calculateHorizonBound(maxTransition);
  createDummyEnd();

  const vector<Vehicle*> vehicles = _problem.getVehicles();
  assert(vehicles.size() == 1);
  Vehicle *v = vehicles[0];

  // JCBTODO: set up alternative activities one per vehicle when we have more than one
  // bool optional = (nbVehicles > 1);

  IloIntervalVarArray activitiesForVehicle(_env);
  IloIntArray types(_env); // the activity types for the transition matrix look-up

  map<string, int>& waypointIndex = _problem.getWaypointIndex();

  // dummy activity corresponding to the starting location
  IloIntervalVar start = createDummyStart(v);
  activitiesForVehicle.add(start);
  types.add(waypointIndex[v->getStartWaypoint()]);

  // *** Missions

  for(Mission *m : _problem.getMissions()) {
    IloIntervalVar act = createActivityForMission(v,m);

    _missions.add(act);
    activitiesForVehicle.add(act);
    types.add(waypointIndex[m->getWaypoint()]);
  }

  // *** Recharge actions

  // Each recharge action is made of 3 actvities: dock, recharge, undock constrained
  // We can have a recharge after each mission (except the last one) and also one 
  // after the start activity - so we have the same number of (optional) recharges as missions

  IloIntervalVarArray dockActs(_env);
  IloIntervalVarArray undockActs(_env);
  for(int i = 0; i < _problem.getMissions().size() + 1; ++i) {
    // need an one more recharge than the number of missions because we may
    // start with a recharge
    IloIntervalVar dockAct = createDockActivity(i);
    dockActs.add(dockAct);
    activitiesForVehicle.add(dockAct);
    Recharge *r = (Recharge *) dockAct.getObject();
    types.add(waypointIndex[r->getWaypoint()]);
    
    IloIntervalVar rechargeAct = createRechargeActivity(i);
    _recharges.add(rechargeAct);
    activitiesForVehicle.add(rechargeAct);
    r = (Recharge *) rechargeAct.getObject();
    types.add(waypointIndex[r->getWaypoint()]);

    IloIntervalVar undockAct = createUndockActivity(i);
    undockActs.add(undockAct);
    activitiesForVehicle.add(undockAct);
    r = (Recharge *) undockAct.getObject();
    types.add(waypointIndex[r->getWaypoint()]);

    // all three activities are present or none of them are
    _model.add(IloPresenceOf(_env, dockAct) == IloPresenceOf(_env, rechargeAct));
    _model.add(IloPresenceOf(_env, dockAct) == IloPresenceOf(_env, undockAct));
  }

  // put dummy end activity on the list for each vehicle
  int endType = waypointIndex.size();
  activitiesForVehicle.add(_endOfAll);
  types.add(endType);

  // ** Sequence constraints

  // create sequence var (used in loop for setting the energy levels)
  _vehicleSequence = IloIntervalSequenceVar(_env, activitiesForVehicle, types);

  // add no overlap
  _model.add(IloNoOverlap(_env, _vehicleSequence, _travelTimes));

  // constrain dummy activities to be at start and end of activity sequence
  _model.add(IloFirst(_env, _vehicleSequence, start));
  _model.add(IloLast(_env, _vehicleSequence, _endOfAll));
 
  assert(_recharges.getSize() == _missions.getSize() + 1);
  for(int i = 0; i < _recharges.getSize(); ++i) { 
    // dock, recharge, undock are always a block
    _model.add(IloPrevious(_env, _vehicleSequence, dockActs[i], _recharges[i]));
    _model.add(IloStartAtEnd(_env, _recharges[i], dockActs[i]));

    _model.add(IloPrevious(_env, _vehicleSequence, _recharges[i], undockActs[i]));
    _model.add(IloStartAtEnd(_env, undockActs[i], _recharges[i]));

#ifdef ONE_TO_ONE_RECHARGES
    if (i < _recharges.getSize() - 1) {
      // each mission has an optional charge immediate after it
      _model.add(IloPrevious(_env, _vehicleSequence, _missions[i], dockActs[i]));
      _model.add(IloPresenceOf(_env, _missions[i]) >= IloPresenceOf(_env, dockActs[i]));
      // start is always present so no need for IloPresenceOf constraint
    }
    else {
      // start activity has an associated (optional) recharge immediately after it
      _model.add(IloPrevious(_env, _vehicleSequence, start, dockActs[i]));
    }
#else
    if (i < _recharges.getSize() - 1) {
      // all recharges are completely ordered
      _model.add(IloBefore(_env, _vehicleSequence, undockActs[i], dockActs[i+1]));
      // break some symmetry by constraining recharges 
      //_model.add(IloPresenceOf(_env, undockActs[i]) >= IloPresenceOf(_env, dockActs[i+1]));
    }
#endif 
   

  }
 
  _activities.add(activitiesForVehicle);

  // *** Energy levels
  //
  // Needs to be done in two steps:
  // 1. create the cumulative function
  // 2. post appropriate constraints on the cumulative function

  // Energy level at the beginning corresponds to the contribution at the end of the
  // dummy start activity
  IloInt startCharge = Discretize(v->getStartCharge());
  _vehicleEnergy += IloStepAtStart(start, startCharge);

  IloIntExprArray missionEnergy(_env);
  // Missions
  for(int i = 0; i < _missions.getSize(); ++i) { // ignore dummy start and end

    IloIntervalVar act = _missions[i];
    Mission *m = (Mission *) act.getObject();

    // set the energy consumption of the activity
    IloInt actConsumption = Discretize(m->getCharge());
    IloIntArray deltaCharges = _inverseDeltaCharge[waypointIndex[m->getWaypoint()]];

    // impact of activity is between the energy of the act itself and the energy required
    // to travel from the maximum distance away (plus the energy of the act). This sets the
    // bounds
    _vehicleEnergy -= IloStepAtStart(act, actConsumption, 
				     actConsumption + maxTransition);

    // create expression of usage of the mission - to be posted as a constraint below
    IloIntExpr energyUsage(_env);
    energyUsage += actConsumption + // energy usage of act itself plus
		    // energy used to move to location of act
                   deltaCharges[IloTypeOfPrevious(_vehicleSequence, act, 0)]; 
    missionEnergy.add(energyUsage);

#ifndef NDEBUG
    cout << "Mission " << m->getName() << " " << Undiscretize(actConsumption) 
	 << " " << Undiscretize(maxTransition) << endl;
#endif
  }


  // Recharges
  IloIntExprArray rechargeEnergy(_env);

  for(int i = 0; i < _recharges.getSize(); ++i) {
    IloIntervalVar dockAct = dockActs[i];
    IloIntervalVar rechargeAct = _recharges[i];
    IloIntervalVar undockAct = undockActs[i];
    Recharge *r = (Recharge *) rechargeAct.getObject();
    assert(rechargeAct.getObject() == dockAct.getObject());
    assert(rechargeAct.getObject() == undockAct.getObject());

    // energy consumption by dockAct
    IloInt startConsumption = Discretize(r->getStartCharge());
    //cout << "Start Charge: " << startConsumption << endl;
    IloIntArray deltaCharges = _inverseDeltaCharge[waypointIndex[r->getWaypoint()]];
 
#ifdef ONE_TO_ONE_RECHARGES
    IloInt transitionCharge;
    if (i < _recharges.getSize() - 1) {
      Mission *m = (Mission*) _missions[i].getObject();
      // since we know that dockActs[i] comes right after _missions[i] (if it exists)
      transitionCharge = deltaCharges[waypointIndex[m->getWaypoint()]];
    }
    else
      transitionCharge = deltaCharges[waypointIndex[v->getStartWaypoint()]];

    IloInt energyAtStart = startConsumption + transitionCharge;
    _vehicleEnergy -= IloStepAtStart(dockAct, energyAtStart, energyAtStart);

#ifndef NDEBUG
    cout << dockAct.getName() << ": -" << energyAtStart << endl;
#endif

#else
    // impact of activity is between the energy of the act itself and the energy required
    // to travel from the maximum distance away (plus the energy of the act). This sets the
    // bounds
    _vehicleEnergy -= IloStepAtStart(dockAct, startConsumption, 
				     startConsumption + maxTransition);

    IloIntExpr energyUsage(_env);
    energyUsage += startConsumption + // energy usage to dock with recharger
                   // energy used to move to location of recharge
                   deltaCharges[IloTypeOfPrevious(_vehicleSequence, dockAct, 0)]; 

    rechargeEnergy.add(energyUsage);

#endif

    // energy production at end
    // At the end of the rechargeAct, we have a increase in the interval of [1, endCharge]
    IloInt endCharge = Discretize(r->getEndCharge());
    _vehicleEnergy += IloStepAtEnd(rechargeAct, 0, endCharge);

#ifndef NDEBUG
    cout << rechargeAct.getName() << ": +" << endCharge << endl;

    cout << "Recharge: " << Undiscretize(startConsumption) 
	 << " " << Undiscretize(endCharge) << endl;
#endif
  }

  // Step 2: Now that the cumul function is fully defined, add the constraints.

  // Missions
  for(int i = 0; i < _missions.getSize(); ++i) { // ignore dummy start and end

    // constraint on the exact contribution at the start
    _model.add(IloHeightAtStart(_missions[i], _vehicleEnergy) == 
	                               -(missionEnergy[i] * IloPresenceOf(_env,_missions[i])));
  }


  // Recharges
  for(int i = 0; i < _recharges.getSize(); ++i) {

#ifndef ONE_TO_ONE_RECHARGES
     
    // constraint on the exact contribution at the start
    _model.add(IloHeightAtStart(dockAct, _vehicleEnergy) == 
	                               -(rechargeEnergy[i] * IloPresenceOf(_env,dockActs[i])));
#endif

    // we now constrain the increase at the end of the rechargeAct to be endCharge
    // by specifying that the energy level during all of the undockAct should 
    // be endCharge

    Recharge *r = (Recharge *) _recharges[i].getObject();
    IloInt endCharge = Discretize(r->getEndCharge());
    _model.add(IloAlwaysIn(_env, _vehicleEnergy, undockActs[i], endCharge, endCharge));
  }


  // Energy variable bounds
  IloInt maxCharge = Discretize(_problem.getMaxCharge());
#ifndef NDEBUG
  cout << "Max Charge: " << Undiscretize(maxCharge) << endl;
#endif
  _model.add(IloAlwaysIn(_env, _vehicleEnergy, 0, _horizon, 0, maxCharge));

#ifdef RECHARGE_LB
  addRechargeBound();
#endif

  return true;
}
 

void Solver::setObjective() {

  //////////////////////////////////////////////////////////////
  // Objective Function (1): Maximize Number of Missions completed
  
  IloNumExprArray missionStatus(_env);
  for (int i = 0; i < _missions.getSize(); i++) 
     missionStatus.add(IloPresenceOf(_env, _missions[i]));

  // maximize number of missions done while minimizing number of recharges
  IloNumExpr numMissions = IloSum(missionStatus); // - (IloSum(rechargeStatus) / (missions.getSize() + 1));


  switch(_problem.getObjective()) {
  case 0: // maximize number of missions
    _obj = IloMaximize(_env,numMissions);
    break;
  case 1: // maximimize number of missions minus number of recharges
    {
      // JCBTODO: Add a lower bound on the number of recharges, given the consumption of
      // the missions (element constraint based on increasingly sorted mission charge)
      IloNumExprArray rechargeStatus(_env);
      for(int i = 0; i < _recharges.getSize(); ++i) 
	rechargeStatus.add(IloPresenceOf(_env, _recharges[i]));
//       _model.add(numMissions == _missions.getSize());
//       _obj = IloMinimize(_env, IloSum(rechargeStatus));
      IloNumExpr weightedMissions = (_recharges.getSize() + 1) * numMissions - 
	IloSum(rechargeStatus);
      _obj = IloMaximize(_env, weightedMissions);
    }
    break;
  case 2:
    _model.add(numMissions == _missions.getSize());
 
    _obj = IloMinimize(_env, IloEndOf(_endOfAll));
    break;

  default:
    cout << "Error - no corresponding objective function: " << _problem.getObjective() << endl;
  }

  _model.add(_obj);

}

void Solver::simpleSolve() {
  IloCP cp(_model);
  cp.setParameter(IloCP::Workers, 1);
  cp.setParameter(IloCP::TimeLimit, _problem.getTimeLimit());
  //cp.setParameter(IloCP::LogVerbosity, IloCP::Verbose);
  cp.setParameter(IloCP::LogVerbosity, IloCP::Normal);
  //cp.setParameter(IloCP::SearchType, IloCP::DepthFirst);

  int solID = 0;
  cp.startNewSearch();
  while(cp.next()) {
    printSolution(cp, solID);
    solID++;
  }

  cp.out() << "*** DONE ***\t" << cp.getStatus() << endl;

  if (cp.getStatus() == IloAlgorithm::Infeasible || cp.getStatus() == IloAlgorithm::Unknown) {
    cout << "No further improving solutions." << endl;
  }
  else {
    printSolution(cp,solID-1, true, true);

//   if (status == IloAlgorithm::Feasible
//   cp.out() << "*** Final solution ***\t" << cp.getObjValue() << " " 
// 	       << float(clock() - _startTime) / CLOCKS_PER_SEC << " " 
// 	       << cp.getStatus() << " " 
// 	       << cp.getInfo(IloCP::NumberOfChoicePoints) << " " 
// 	       << cp.getInfo(IloCP::NumberOfFails) << endl;

//   for(IloIntervalVar v = cp.getFirst(_vehicleSequence);  
//       v.getImpl()!=0; v = cp.getNext(_vehicleSequence, v)) {
//     cp.out() << v.getName() << " [" << Undiscretize(cp.getStart(v)) << " -- "
// 	     << Undiscretize(cp.getLength(v)) << " --> " 
// 	     << Undiscretize(cp.getEnd(v)) << "]" << endl;
//     //cp.out() << cp.domain(v) << endl;
//   }

    for (IloInt i=0; i < cp.getNumberOfSegments(_vehicleEnergy); ++i)
      cp.out() << "["  << Undiscretize(cp.getSegmentStart(_vehicleEnergy, i))
	       << ","  << Undiscretize(cp.getSegmentEnd(_vehicleEnergy, i))
	       << "): " << Undiscretize(cp.getSegmentValue(_vehicleEnergy, i))
	       << endl;
  }

  cp.end();
}


void Solver::solve() {
  IloCP cp(_model);
  cp.setParameter(IloCP::Workers, 1);
  cp.setParameter(IloCP::TimeLimit, _problem.getTimeLimit());
  //cp.setParameter(IloCP::LogVerbosity, IloCP::Verbose);
  cp.setParameter(IloCP::LogVerbosity, IloCP::Normal);
  //cp.setParameter(IloCP::SearchType, IloCP::DepthFirst);

  IloIntervalSequenceVarArray sequenceVars(_env);
  sequenceVars.add(_vehicleSequence);
  IloSearchPhaseArray phaseArray(_env);
  IloIntVarArray presenceOf(_env);
  IloIntVarArray presenceOfMissions(_env);
  IloIntVarArray presenceOfRecharges(_env);
  for(int i = 0; i < _missions.getSize(); ++i) {
    IloIntVar p(_env, 0, 1);
    _model.add(p == IloPresenceOf(_env,_missions[i]));
    presenceOfMissions.add(p);
    presenceOf.add(p);
  }
  for(int i = 0; i < _recharges.getSize(); ++i) {
    IloIntVar p(_env, 0, 1);
    _model.add(p == IloPresenceOf(_env,_recharges[i]));
    presenceOfRecharges.add(p);
    presenceOf.add(p);
  }


  phaseArray.add(IloSearchPhase(_env, sequenceVars));
  phaseArray.add(IloSearchPhase(_env, presenceOf));
  cp.setSearchPhases(phaseArray);

  int solID = 0;
  IloInt nbMissionsAndRecharges = presenceOf.getSize();
  bool noSolution = true;
  while(noSolution && (nbMissionsAndRecharges > 0)) {

    IloConstraint heuristicSol = (IloSum(presenceOf) >= nbMissionsAndRecharges);
    _model.add(heuristicSol);
    if (cp.solve()) {
      printSolution(cp, solID, false);
      solID++;
      noSolution = false;
    }
    else
      nbMissionsAndRecharges -= 2;

    _model.remove(heuristicSol);
  }

  if (noSolution) {
    cout << "No feasible solutions!" << endl;
    return;
  }

  //printSolution(cp, solID);

  // we have found a feasible solution with nbMissionsAndRecharges being
  // the number of missions and recharges done
  IloInt nbMissions = 0;
  for(int i = 0; i < _missions.getSize(); ++i) 
    nbMissions += cp.isPresent(_missions[i]);
  IloInt nbRecharges = 0;
  for(int i = 0; i < _recharges.getSize(); ++i) 
    nbRecharges += cp.isPresent(_recharges[i]);

  cp.out() << "Missions: " << nbMissions << "\tRecharges: " << nbRecharges << endl;

  _model.add(IloSum(presenceOfMissions) >= nbMissions);

  bool solution = true;
  nbRecharges--;
  while(solution && (nbRecharges > 0)) {
    
    IloConstraint heuristicSol = (IloSum(presenceOfRecharges) == nbRecharges);
    _model.add(heuristicSol);
    if (cp.solve()) {
      printSolution(cp, solID, false);
      solID++;

      nbMissions = 0;
      for(int i = 0; i < _missions.getSize(); ++i) 
	nbMissions += cp.isPresent(_missions[i]);
      cp.out() << "Missions: " << nbMissions << "\tRecharges: " << nbRecharges << endl;

      nbRecharges--;
    }
    else
      solution = false;

    _model.remove(heuristicSol);
  }

  if (nbRecharges == 0 || cp.getStatus() == IloAlgorithm::Infeasible) 
    cout << "No more feasible solutions! Last solution is optimal." << endl;
  else
    cout << "Timed out" << endl;

//   cp.clear();
//   cp.extract(_model);
//   cp.startNewSearch();
//   while(cp.next()) {
//     printSolution(cp, solID);
//     solID++;

//     nbMissions = 0;
//     for(int i = 0; i < _missions.getSize(); ++i) 
//       nbMissions += cp.isPresent(_missions[i]);
//     nbRecharges = 0;
//     for(int i = 0; i < _recharges.getSize(); ++i) 
//       nbRecharges += cp.isPresent(_recharges[i]);
//     cp.out() << "Missions: " << nbMissions << "\tRecharges: " << nbRecharges << endl;
//   }
//   cout << cp.getStatus() << endl;

//   if (cp.getStatus() == IloAlgorithm::Infeasible || cp.getStatus() == IloAlgorithm::Unknown) {
//     cout << "No further improving solutions." << endl;
//   }
//   else {
//     printSolution(cp,solID-1);
//     cp.out() << "Missions: " << nbMissions << "\tRecharges: " << nbRecharges << endl;

// //   if (status == IloAlgorithm::Feasible
// //   cp.out() << "*** Final solution ***\t" << cp.getObjValue() << " " 
// // 	       << float(clock() - _startTime) / CLOCKS_PER_SEC << " " 
// // 	       << cp.getStatus() << " " 
// // 	       << cp.getInfo(IloCP::NumberOfChoicePoints) << " " 
// // 	       << cp.getInfo(IloCP::NumberOfFails) << endl;
 
// //   for(IloIntervalVar v = cp.getFirst(_vehicleSequence);  
// //       v.getImpl()!=0; v = cp.getNext(_vehicleSequence, v)) {
// //     cp.out() << v.getName() << " [" << Undiscretize(cp.getStart(v)) << " -- "
// // 	     << Undiscretize(cp.getLength(v)) << " --> " 
// // 	     << Undiscretize(cp.getEnd(v)) << "]" << endl;
// //     //cp.out() << cp.domain(v) << endl;
// //   }

//     for (IloInt i=0; i < cp.getNumberOfSegments(_vehicleEnergy); ++i)
//       cp.out() << "["  << Undiscretize(cp.getSegmentStart(_vehicleEnergy, i))
// 	       << ","  << Undiscretize(cp.getSegmentEnd(_vehicleEnergy, i))
// 	       << "): " << Undiscretize(cp.getSegmentValue(_vehicleEnergy, i))
// 	       << endl;
//   }

  cp.end();
}

void Solver::printSolution(const IloCP& cp, int solID, bool showObj, bool done) {
  if (done) 
    cp.out() << "**** FINAL ";

  cp.out() << "**** Solution #" << solID;

  IloInt nbMissions = 0;
  for(int i = 0; i < _missions.getSize(); ++i) 
    nbMissions += cp.isPresent(_missions[i]);
  IloInt nbRecharges = 0;
  for(int i = 0; i < _recharges.getSize(); ++i) 
    nbRecharges += cp.isPresent(_recharges[i]);

  if (showObj)
    cp.out() << " " << cp.getObjValue(); 

  cp.out() << "\tMissions: " << nbMissions << "\tRecharges: " << nbRecharges << "\t";

  cp.out() << " " << float(clock() - _startTime) / CLOCKS_PER_SEC << " " 
	   << cp.getStatus() << " " 
	   << cp.getInfo(IloCP::NumberOfChoicePoints) << " " 
	   << cp.getInfo(IloCP::NumberOfFails) << endl;
  
  for(IloIntervalVar v = cp.getFirst(_vehicleSequence);  
      v.getImpl()!=0; v = cp.getNext(_vehicleSequence, v)) {
    cp.out() << v.getName() << " [" << Undiscretize(cp.getStart(v)) << " -- "
	     << Undiscretize(cp.getLength(v)) << " --> " 
	     << Undiscretize(cp.getEnd(v)) << "]" << endl;
  }
}
