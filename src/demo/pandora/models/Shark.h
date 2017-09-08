#ifndef PANDORA_MODELS_SHARK_H
#define PANDORA_MODELS_SHARK_H

#include "dpengine/entities/Entity.h"

namespace DreadedPE
{
	class Camera;
	class SceneNode;
	class Terrain;
	class SceneManager;
	class Material;
	class ShaderInterface;
	class Line;
}

class Shark : public DreadedPE::Entity
{
public:
	Shark(DreadedPE::SceneNode* parent, const glm::mat4& transformation, DreadedPE::SceneManager& scene_manager);
	
	void init(DreadedPE::Material& material, DreadedPE::ShaderInterface& shader);
	
	virtual void prepare(float dt);
	void onCollision(const DreadedPE::CollisionInfo& collision_info);

	void setWaypoints(const std::vector<glm::vec3>& waypoints) { waypoints_ = waypoints; }

private:
	void transition(DreadedPE::SceneNode* new_node, glm::vec3& local_location);

	float height_;
	float velocity_;
	float desired_velocity_;
	float max_velocity_;
	float rotation_speed_;
	
	glm::vec3 desired_direction_;
	
	float pitch_, yaw_, roll_;
	float desired_pitch_, desired_yaw_;
	
	unsigned int nr_collisions_checked_;

	std::vector<glm::vec3> waypoints_;
	unsigned int current_waypoint_;

	DreadedPE::Line* collision_line_;
};

#endif
