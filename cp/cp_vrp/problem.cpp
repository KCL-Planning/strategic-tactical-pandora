/* ------------------------------------------------------------
 CP Code for MRTA Retirement Home Problem (Kyle model)
 Purpose: Primary functions
 ------------------------------------------------------------ */

#include "problem.h"
#include <math.h>
#include <ilcp/cp.h>
#include <ilcplex/ilocplex.h>
#include <algorithm>

ILOSTLBEGIN

#define NAME_SIZE 80

ostream& operator<<(ostream& os, const Vehicle& v) {
  os << v._name << " " << v._charge << " " << v._waypoint;
  return os;
}
 
ostream& operator<<(ostream& os, const Mission& m) {
  os << m._name << " " << m._duration << " " << m._charge 
     << " " << m._waypoint << " " << m._deadline;
  return os;
}

ostream& operator<<(ostream& os, const Recharge& r) {
  os << r._waypoint << " " << r._duration << " " << r._startCharge 
     << " " << r._endCharge;
  return os;
}

ifstream& Problem::getNextLine(ifstream& file, string& s) {
  while(getline(file,s) && (s[0] == '#'))
    ;
  return file;

//   ifstream = getline(file,s);
//   while(ret && (s[0] == '#')) // skip comments
//     ret = getline(file,s); 
//   return ret;

}

void Problem::parseVehicle(string line) {
  char vName[NAME_SIZE];
  float charge;
  char waypoint[NAME_SIZE];
  sscanf(line.c_str(),"%s %f %s", vName, &charge, waypoint);
  _vehicles.push_back(new Vehicle(vName, charge, waypoint));
}

void Problem::parseMission(string line) {
  char vName[NAME_SIZE];
  float duration;
  float charge;
  char waypoint[NAME_SIZE];
  float deadline;
  sscanf(line.c_str(),"%s %f %f %s %f", vName, &duration, &charge, 
	 waypoint, &deadline);
  _missions.push_back(new Mission(vName, duration, charge, 
				    waypoint, deadline));
}

void Problem::parseRecharge(string dock, string recharge, string undock) {
  char name[NAME_SIZE];
  float dock_duration;
  float start_charge;
  
  sscanf(dock.c_str(),"%s %f %f", name, &dock_duration, &start_charge);
  assert(strcmp(name,"dock") == 0);

  char waypoint[NAME_SIZE];
  float duration;
  float end_charge;
  sscanf(recharge.c_str(),"%s %s %f %f", 
	 name, waypoint, &duration, &end_charge);
  assert(strcmp(name,"recharge") == 0);

  float undock_duration;
  sscanf(undock.c_str(),"%s %f", name, &undock_duration);
  assert(strcmp(name,"undock") == 0);

  _recharges.push_back(new Recharge(waypoint, 
				    dock_duration, duration, undock_duration, 
				    start_charge, end_charge));
}

void Problem::parseWaypointIndex(string line) {
  // gives an index to each waypoint so we can index the 2D matrix
  char wp1[NAME_SIZE], wp2[NAME_SIZE];
  float distance; // ignored
  sscanf(line.c_str(),"%s %s %f", wp1, wp2, &distance);

  //cout << wp1 << " " <<  _waypointIndex.size() << endl;

  if (_waypointIndex.find(wp1) == _waypointIndex.end())
    _waypointIndex[wp1] = _waypointIndex.size();

  //cout << wp1 << " " <<  _waypointIndex.size() << endl;
  if (_waypointIndex.find(wp2) == _waypointIndex.end())
    _waypointIndex[wp2] = _waypointIndex.size();
}
  
void Problem::parseAllDistances(vector<string> allDistances) {
  // processes all distances into a distance matrix

//   for(map<string,int>::iterator it = _waypointIndex.begin();
//       it != _waypointIndex.end();
//       it++) {
//     cout << it->first << " " << it->second << endl;
//   }


  // create 2D distance array
  int nbWaypoints = _waypointIndex.size();
  _distanceMatrix = new float*[nbWaypoints];
  for(int i = 0; i < nbWaypoints; ++i) {
    _distanceMatrix[i] = new float[nbWaypoints];
    for(int j = 0; j < nbWaypoints; ++j) 
      _distanceMatrix[i][j] = 0;
  }

//   for(int i = 0; i < _waypointIndex.size(); ++i) {
//     for(int j = 0; j < _waypointIndex.size(); ++j) 
//       cout << "\t" << _distanceMatrix[i][j];
//     cout << endl;
//   }

  // for each entry in allDistances, set the distance
  for (string s : allDistances) {
    char wp1[NAME_SIZE], wp2[NAME_SIZE];
    float distance; 
    sscanf(s.c_str(),"%s %s %f", wp1, wp2, &distance);
    //cout << s << endl;
    int index1 = _waypointIndex.find(wp1)->second;
    //cout << index1 << endl;
    int index2 = _waypointIndex.find(wp2)->second;
    //cout << index2 << endl;

    assert(_distanceMatrix[index1][index2] == 0);
    _distanceMatrix[index1][index2] = distance;
  }
}

bool Problem::readInstanceData() {
  // open and read the data in _problemFile, populating the variables


  ifstream datafile(_problemFile);
  if (!datafile.is_open()) {
    cout << "Unable to open file: " << _problemFile;
    return false;
  }

  string line;
  if (!getNextLine(datafile,line)) {
    datafile.close();
    return false;
  }

  // ** Vehicles **
  char var_name[NAME_SIZE];
  int nbVehicles;
  sscanf(line.c_str(),"%s %d", var_name, &nbVehicles);
  assert(strcmp(var_name,"nb_vehicles") == 0);
  
  for(int i = 0; i < nbVehicles; ++i) {
    if (!getNextLine(datafile,line)) {
      datafile.close();
      return false;
    }

    parseVehicle(line);
  }
      
  if (!getNextLine(datafile,line)) {
    datafile.close();
    return false;
  }
  
  // ** Missions **
  int nbMissions;
  sscanf(line.c_str(),"%s %d", var_name, &nbMissions);
  assert(strcmp(var_name,"nb_missions") == 0);

  for(int i = 0; i < nbMissions; ++i) {
    if (!getNextLine(datafile,line)) {
      datafile.close();
      return false;
    }

    parseMission(line);
  }
      
  if (!getNextLine(datafile,line)) {
    datafile.close();
    return false;
  }

  // ** Recharges **
  int nbRecharges;
  sscanf(line.c_str(),"%s %d", var_name, &nbRecharges);
  assert(strcmp(var_name,"nb_recharges") == 0);

  for(int i = 0; i < nbRecharges; ++i) {
    // each recharge is actually 3 actions specified on separate lines
    string dock_str;
    if (!getNextLine(datafile,dock_str)) {
      datafile.close();
      return false;
    }

    string recharge_str;
    if (!getNextLine(datafile,recharge_str)) {
      datafile.close();
      return false;
    }
    
    string undock_str;
    if (!getNextLine(datafile,undock_str)) {
      datafile.close();
      return false;
    }
    
    parseRecharge(dock_str,recharge_str,undock_str);
   
  }
      
  if (!getNextLine(datafile,line)) {
    datafile.close();
    return false;
  }

  // ** Distances **
  int nbDistances;
  sscanf(line.c_str(),"%s %d", var_name, &nbDistances);
  assert(strcmp(var_name,"nb_distances") == 0);

  vector<string> allDistances;
  for(int i = 0; i < nbDistances; ++i) {
    if (!getNextLine(datafile,line)) {
      datafile.close();
      return false;
    }

    parseWaypointIndex(line);
    allDistances.push_back(string(line));
  }

  parseAllDistances(allDistances);

  if (!getNextLine(datafile,line)) {
    datafile.close();
    return false;
  }
  
  // ** Travel charge **
  sscanf(line.c_str(),"%s %f", var_name, &_travelChargeCoeff);
  assert(strcmp(var_name,"delta_charge_coeff") == 0);

  if (!getNextLine(datafile,line)) {
    datafile.close();
    return false;
  }
  
  // ** Travel duration **
  sscanf(line.c_str(),"%s %f", var_name, &_travelDurationCoeff);
  assert(strcmp(var_name,"travel_duration_coeff") == 0);

  datafile.close();
  
  return true;
}

void Problem::dumpData() {
  cout << "Problem File: " << _problemFile << endl;
  cout << "Time Limit: " << _timeLimit << endl;

  cout << "Num Vehicles: " << _vehicles.size() << endl;
  for(Vehicle *v : _vehicles)
    cout << "\t" << *v << endl;

  cout << "Num Missions: " << _missions.size() << endl;
  for(Mission *m : _missions)
    cout << "\t" << *m << endl;

  cout << "Num Recharges: " << _recharges.size() << endl;
  for(Recharge *r : _recharges)
    cout << "\t" << *r << endl;

  cout << "Num Waypoints: " << _waypointIndex.size() << endl;
  for(unsigned int i = 0; i < _waypointIndex.size(); ++i) {
    for(unsigned int j = 0; j < _waypointIndex.size(); ++j) 
      cout << "\t" << _distanceMatrix[i][j];
    cout << endl;
  }

  cout << "Delta charge for travel: " << _travelChargeCoeff << endl;
}

float Problem::getMaxCharge() const {
  float maxCharge = 0;
  for(Recharge *r : _recharges) {
    if (r->getEndCharge() > maxCharge)
      maxCharge = r->getEndCharge();
  }

  return maxCharge;
}
