#ifndef DEMO_PANDORA_ONTOLOGY_VALVE_GOAL_H
#define DEMO_PANDORA_ONTOLOGY_VALVE_GOAL_H

#include "Goal.h"

class Structure;
class Valve;

class ValveGoal : public Goal
{
public:
	ValveGoal(Structure& structure, Valve& valve, double start_time, double deadline, float valve_angle);
	virtual ~ValveGoal();
	
	const Valve& getValve() const { return *valve_; }
	double getStartTime() const { return start_time_; }
	double getDeadline() const { return deadline_; }
	float getValveAngle() const { return valve_angle_; }
	void getStrategicPoints(std::vector<glm::vec3>& points) const;
	
	void reset();
	
private:
	Valve* valve_;
	double start_time_;
	double deadline_;
	float valve_angle_;
};

#endif
