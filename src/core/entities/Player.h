#ifndef CORE_AGENTS_PLAYER_H
#define CORE_AGENTS_PLAYER_H

#include <glm/glm.hpp>
#include <sstream>

#include "Entity.h"

//class Camera;
class SceneNode;
class Terrain;
class SceneManager;

class Player : public Entity
{
public:
	Player(SceneNode* parent, const glm::mat4& transformation, float height, SceneNode& scene_node, const Terrain& terrain, SceneManager& scene_manager);

	virtual ~Player();

	void prepare(float dt);

	//const Camera& getCamera() const { return *camera_; }

	void onCollision(const CollisionInfo& collision_info);

	std::stringstream& getString() { return ss_; }

	unsigned int getScore() const { return score_; }
	unsigned int getDeaths() const { return deaths_; }

	// Debug.
	float getPrepareTime() const { return prepare_time_; }
	float getCollisionDetectionTime() const { return collision_detection_time_; }
	void resetTimers() { prepare_time_ = 0; collision_detection_time_ = 0; }

	unsigned int getCollisionChecksPerformed() const { return nr_collisions_checked_; }

protected:

	void transition(SceneNode* new_node, glm::vec3& local_location);

	float height_;

	//Camera* camera_;
	const Terrain* terrain_;
	
	float y_velocity_;

	// Local position and orientation.
	float pitch_, yaw_, roll_;

	float parent_pitch_, parent_yaw_, parent_roll_;

	std::stringstream ss_;

	SceneNode* default_entity_;

	unsigned int score_;
	unsigned int deaths_;

	unsigned int nr_collisions_checked_;

	// Debug.
	float prepare_time_, collision_detection_time_;
};

#endif
