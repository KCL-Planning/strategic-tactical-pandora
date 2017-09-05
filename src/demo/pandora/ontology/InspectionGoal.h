#ifndef DEMO_PANDORA_ONTOLOGY_INSPECTION_GOAL_H
#define DEMO_PANDORA_ONTOLOGY_INSPECTION_GOAL_H
#include <vector>

#include "Goal.h"

class Structure;
class InspectionPoint;

class InspectionGoal : public Goal
{
public:
	InspectionGoal(Structure& structure);
	
	InspectionGoal(Structure& structure, const std::vector<InspectionPoint*>& inspection_points);
	
	virtual ~InspectionGoal();
	
	void addInspectionPoint(InspectionPoint& inspection_point) { inspection_points_.push_back(&inspection_point); }
	
	void setInspectionPoints(const std::vector<InspectionPoint*>& inspection_points) { inspection_points_ = inspection_points; }
	
	const std::vector<InspectionPoint*>& getInspectionPoints() const { return inspection_points_; }
	
	void removeInspectionPoint(InspectionPoint& inspection_point);
	
	bool isEnabled() const { return !inspection_points_.empty(); }
	
	void getStrategicPoints(std::vector<glm::vec3>& points) const;
	
private:
	std::vector<InspectionPoint*> inspection_points_;
};

#endif
