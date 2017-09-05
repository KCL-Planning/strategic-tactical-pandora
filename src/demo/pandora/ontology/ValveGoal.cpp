#include "ValveGoal.h"
#include "../structures/Valve.h"
#include "../structures/Structure.h"

ValveGoal::ValveGoal(Structure& structure, Valve& valve, double start_time, double deadline, float valve_angle)
	: Goal(structure), valve_(&valve), start_time_(start_time), deadline_(deadline), valve_angle_(valve_angle)
{
	
}

ValveGoal::~ValveGoal()
{

}

void ValveGoal::getStrategicPoints(std::vector<glm::vec3>& points) const
{
	points.push_back(valve_->getStructure().getInteractLocation());
}

void ValveGoal::reset()
{
	valve_angle_ += 180;
}
