#ifndef DEMO_PANDORA_AUV_H
#define DEMO_PANDORA_AUV_H

#ifndef _WIN32
#include <ros/ros.h>
#endif

#include <glm/glm.hpp>

#include "../../core/entities/Entity.h"
#include "../../core/gui/events/ButtonPressedListener.h"

class SceneNode;
class Propeller;
class RRT;
class SceneManager;
class GPUParticleEmitter;
class GPUParticleComputerShader;
class GPUParticleDrawShader;
class Texture;
class OctomapBuilder;
class BillBoard;
class Waypoint;
class RobotHand;

class AUV : public Entity
{
public:
	AUV(SceneNode* parent, const glm::mat4& transformation, SceneManager& scene_manager, Texture& texture, const std::string& frame_name);

	virtual ~AUV();

	void prepare(float dt);

	void onCollision(const CollisionInfo& collision_info);

	//void addPropeller(Propeller& propeller);
	
	void setDirection(const glm::vec3& direction) { desired_direction_ = direction; }
	void setVelocity(float velocity) { desired_velocity_ = velocity; }
	
	void setDesiredOrientation(float yaw, float pitch);
	void unsetDesiredOrientation();
	
	const glm::vec3& getDesiredDirection() const { return desired_direction_; }
	
	SceneNode& getAUVNode() const { return *auv_node_; }
	SceneNode& getAUVModel() const { return *model_node_; }
	
	void setLightOn(bool light_is_on) { light_is_on_ = light_is_on; }
	bool isLightOn() const { return light_is_on_; }
	
	float getYaw() const { return yaw_; }
	float getDesiredYaw() const { return desired_yaw_; }
	
	void setBillBoard(BillBoard& bill_board);
	void setBillBoardUVs(const std::vector<glm::vec2>& uv);
	
	Waypoint& getWaypoint() const { return *auv_waypoint_; }
	
	RobotHand& getRobotHand() const { return *robot_hand_; }

protected:
	
	// Local position and orientation.
	float pitch_, yaw_, roll_;
	float velocity_;
	float max_velocity_;
	
	float desired_velocity_;
	glm::vec3 desired_direction_;
	
	bool face_forward_; // If true then the AUV will move where it is pointing.
	
	SceneNode* auv_node_; // The root of the AUV shape that is a child of this entity.
	SceneNode* model_node_; // The node that actually holds the model.
	
	float total_time_;

	Propeller* down_thrust_;
	Propeller* down_thrust2_;
	Propeller* side_thrust_;
	Propeller* forward_thrust_;
	Propeller* forward_thrust2_;
	
	bool light_is_on_;
	
	float desired_yaw_, desired_pitch_; // If face_forward_ is false then these parameters determine where the AUV should be pointing.
	
	BillBoard* status_label_;
	float bill_board_time_;
	
	Waypoint* auv_waypoint_;
	
	RobotHand* robot_hand_;
};

#endif
