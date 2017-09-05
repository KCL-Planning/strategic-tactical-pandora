#include "InspectionGoal.h"
#include "InspectionPoint.h"

InspectionGoal::InspectionGoal(Structure& structure)
	: Goal(structure)
{
	
}

InspectionGoal::InspectionGoal(Structure& structure, const std::vector<InspectionPoint*>& inspection_points)
	: Goal(structure), inspection_points_(inspection_points)
{
	
}

InspectionGoal::~InspectionGoal()
{

}

void InspectionGoal::removeInspectionPoint(InspectionPoint& inspection_point)
{
	for (std::vector<InspectionPoint*>::iterator i = inspection_points_.begin(); i != inspection_points_.end(); ++i)
	{
		if (*i == &inspection_point)
		{
			inspection_points_.erase(i);
			return;
		}
	}
}

void InspectionGoal::getStrategicPoints(std::vector<glm::vec3>& points) const
{
	for (std::vector<InspectionPoint*>::const_iterator i = inspection_points_.begin(); i != inspection_points_.end(); ++i)
	{
		points.push_back((*i)->getVisiblePoint());
	}
}
