/* ------------------------------------------------------------
 CP Code for MRTA Retirement Home Problem (Kyle model)
 Purpose: Header for params/functions (needs organzing)
 ------------------------------------------------------------ */

#ifndef _PROBLEM_H_
#define _PROBLEM_H_

#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <math.h>
#include <ilcp/cp.h>

ILOSTLBEGIN

class Vehicle {
private:
  string _name;
  float _charge;
  string _waypoint;

public:
  Vehicle(string n, float c, string w) : _name(n), _charge(c), _waypoint(w) {}

  friend ostream& operator<<(ostream&, const Vehicle& v);  

  const string& getName() const { return _name; }
  const string& getStartWaypoint() const { return _waypoint; }

  float getStartCharge() const { return _charge; }

};

class Mission {
private:
  string _name;
  float _duration;
  float _charge;
  string _waypoint;
  float _deadline;

public:
  Mission(string n, float dur, float c, string w, float d) : _name(n), _duration(dur), _charge(c), _waypoint(w), _deadline(d) {}

  friend ostream& operator<<(ostream&, const Mission& v);    

  const string& getName() const { return _name; }
  float getDuration() const { return _duration; }
  const string& getWaypoint() const { return _waypoint; }
  float getDeadline() const { return _deadline; }
  float getCharge() const { return _charge; }

};

class Recharge {
private:
  string _waypoint;
  float _dockDuration;
  float _duration;
  float _undockDuration;
  float _startCharge;
  float _endCharge;

public:
  Recharge(string w, float dock_dur, float dur, float undock_dur, float sc, float ec) 
    : _waypoint(w), _dockDuration(dock_dur), _duration(dur), _undockDuration(undock_dur),
      _startCharge(sc), _endCharge(ec) {}

  friend ostream& operator<<(ostream&, const Recharge& v);  

  const string& getWaypoint() const { return _waypoint; }
  float getDockDuration() const { return _dockDuration; }
  float getDuration() const { return _duration; }
  float getUndockDuration() const { return _undockDuration; }

  float getStartCharge() const { return _startCharge; }
  float getEndCharge() const { return _endCharge; }
};

// Class for reading the problem
class Problem {

private:
  string _problemFile;
  double _timeLimit;
  int _objective;

  vector<Vehicle*> _vehicles;
  vector<Mission*> _missions;
  vector<Recharge*> _recharges;

  map<string,int> _waypointIndex;
  float **_distanceMatrix;

  float _travelChargeCoeff;
  float _travelDurationCoeff;



  ifstream& getNextLine(ifstream&, string&);
  void parseVehicle(string);
  void parseMission(string);
  void parseRecharge(string,string,string);

  void parseWaypointIndex(string);
  void parseAllDistances(vector<string>);

public:
  Problem(string probFile, double timeLimit, int obj) :
    _problemFile(probFile),
    _timeLimit(timeLimit),
    _objective(obj),
    _distanceMatrix(NULL), 
    _travelChargeCoeff(0),
    _travelDurationCoeff(0)
  { }
  ~Problem() {}

  bool readInstanceData();
  void dumpData();

  const vector<Vehicle*>& getVehicles() const { return _vehicles; }
  const vector<Mission*>& getMissions() const { return _missions; }
  const vector<Recharge*>& getRecharges() const { return _recharges;}

  map<string,int>& getWaypointIndex() { return _waypointIndex; }
  float getDistance(int i, int j) const { return _distanceMatrix[i][j]; }
  double getTimeLimit() const { return _timeLimit; }

  IloInt getTravelChargeCoeff() const {
    assert(ceilf(_travelChargeCoeff) == _travelChargeCoeff); // is it integer?
    return (IloInt) _travelChargeCoeff;
  }
  IloInt getTravelDurationCoeff() const {
    assert(ceilf(_travelDurationCoeff) == _travelDurationCoeff); // is it integer?
    return (IloInt) _travelDurationCoeff;
  }

  IloInt getObjective() const { return _objective; }

  float getMaxCharge() const;
};

#endif
