#ifndef DEMO_PANDORA_ONTOLOGY_INSPECTION_POINT_H
#define DEMO_PANDORA_ONTOLOGY_INSPECTION_POINT_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <string>
#include <sstream>

#include "Goal.h"
#include "Pose.h"

class Pillar;
class MissionSite;

class InspectionPoint
{
public:
	
	/**
	 * Create an inspection point, either for a pillar or for something else.
	 * @param pose The location and orientation of the inspection point. The orientation will decide
	 * where the waypoint will be added that can see the inspection point.
	 * @param pillar The pillar that this inspection point can see (can be NULL).
	 */
	InspectionPoint(const Pose& pose, Pillar* pillar = NULL)
		: pose_(pose), pillar_(pillar)
	{
		static int global_inspection_point_id_ = 0;
		std::stringstream ss;
		ss << "inspection_point_" << global_inspection_point_id_;
		id_ = ss.str();
		global_inspection_point_id_++;
		glm::vec4 visible_point(0, 0, -5, 1);
		
		glm::mat4 rotation_matrix(1.0f);
		rotation_matrix = glm::rotate(rotation_matrix, pose.yaw_, glm::vec3(0, 1, 0));
		rotation_matrix = glm::rotate(rotation_matrix, pose.pitch_, glm::vec3(1, 0, 0));
		
		visible_point = rotation_matrix * visible_point;
		//glm::fquat rot(glm::vec3(pose.pitch_, pose.yaw_, 0));
		
		//visible_from_ = glm::rotate(rot, visible_from_);
		
		visible_from_ = glm::vec3(visible_point) + glm::vec3(pose.x_, pose.y_, pose.z_);
	}
	
	const std::string& getId() const { return id_; }
	const Pose& getPose() const { return pose_; }
	const glm::vec3& getVisiblePoint() const { return visible_from_; }
	Pillar* getPillar() const { return pillar_; }
	
private:
	std::string id_;
	Pose pose_;
	glm::vec3 visible_from_;
	
	Pillar* pillar_;  // The pillar it can see (or NULL if it cannot see any pillar).
};

#endif
