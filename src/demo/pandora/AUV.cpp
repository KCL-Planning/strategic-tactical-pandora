#define GLX_GLXEXT_LEGACY //Must be declared so that our local glxext.h is picked up, rather than the system one
#define GLFW_NO_GLU // Do not allow GL/glfw to include the gl.h header

#include "GL/glew.h"
#include "GL/glfw.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <time.h>

#include <iostream>

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "AUV.h"
#include "Propeller.h"

#include "../../core/entities/camera/Camera.h"
#include "../../core/scene/SceneNode.h"
#include "../../core/scene/SceneManager.h"
#include "../../shapes/terrain.h"
#include "../../shapes/Cube.h"
#include "../../core/collision/BoxCollision.h"
#include "../../core/collision/CollisionInfo.h"
#include "../../core/scene/SceneLeafModel.h"
#include "../../core/scene/Material.h"
#include "../../core/scene/portal/Region.h"
#include "../../core/particles/GPUParticleEmitter.h"
#include "../../core/particles/GPUParticleComputerShader.h"
#include "../../core/particles/GPUParticleDrawShader.h"
#include "../../core/math/Math.h"
#include "../../core/loaders/WavefrontLoader.h"
#include "../../core/shaders/BasicShadowShader.h"
#include "../../core/entities/behaviours/HoverBehaviour.h"
#include "../../core/texture/TargaTexture.h"
#include "../../core/particles/Particle.h"

#include "shaders/CausticShader.h"
#include "gui/BillBoard.h"
#include "Waypoint.h"
#include "models/RobotHand.h"

#ifndef _WIN32
#include "RRT.h"
#endif

AUV::AUV(SceneNode* parent, const glm::mat4& transformation, SceneManager& scene_manager, Texture& texture, const std::string& frame_name)
	: Entity(scene_manager, parent, transformation, OBSTACLE, frame_name), pitch_(0.0f), yaw_(0.0f), roll_(0.0f), desired_velocity_(0), desired_direction_(glm::vec3(0, 0, -1)), face_forward_(true), desired_yaw_(0), desired_pitch_(0), light_is_on_(false), status_label_(NULL), bill_board_time_(0)
{
	velocity_ = 0.0f;
	BoxCollision* bc = new BoxCollision(*this, 1.2f, 1.2f, 1.5f);
	addCollision(*bc, *SceneNode::bright_material_, BasicShadowShader::getShader());
	max_velocity_ = 16.0f;
	total_time_ = 0;
	
	std::stringstream ss;
	ss << "wp_" << getName();
	auv_waypoint_ = new Waypoint(ss.str(), getGlobalLocation());
	
	// Add a node with a 'wavy' behaviour.
	auv_node_ = new SceneNode(*scene_manager_, this, glm::mat4(1.0f));
	HoverBehaviour* hover_behaviour = new HoverBehaviour(*auv_node_);
	auv_node_->addBehaviour(*hover_behaviour);
	
	model_node_ = new SceneNode(*scene_manager_, auv_node_, glm::mat4(1.0f));

	Shape* auv_model = WavefrontLoader::importShape("data/models/Pandora/misc/AUV.obj");
	Shape* propeller_model = WavefrontLoader::importShape("data/models/Pandora/misc/propeller.obj");
	
	MaterialLightProperty* sub_ambient = new MaterialLightProperty(0.8, 0.8, 0.8, 1.0);
	MaterialLightProperty* sub_diffuse = new MaterialLightProperty(0.6, 0.6, 0.6, 1.0);
	MaterialLightProperty* sub_specular = new MaterialLightProperty(0.3, 0.3, 0.3, 1.0);
	MaterialLightProperty* sub_emmisive = new MaterialLightProperty(0.5, 0.5, 0.5, 1.0);

	Material* sub_material = new Material(*sub_ambient, *sub_diffuse, *sub_specular, *sub_emmisive);
	sub_material->add2DTexture(texture);

	SceneLeafModel* auv_model_leaf = new SceneLeafModel(*model_node_, NULL, *auv_model, *sub_material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	
	down_thrust_ = new Propeller(*scene_manager_, model_node_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.28f, -0.55f)));
	SceneLeafModel* down_thrust_model_leaf = new SceneLeafModel(*down_thrust_, NULL, *propeller_model, *sub_material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	
	down_thrust2_ = new Propeller(*scene_manager_, model_node_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.28f, 0.39f)));
	SceneLeafModel* down_thrust_model_leaf2 = new SceneLeafModel(*down_thrust2_, NULL, *propeller_model, *sub_material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	
	SceneNode* side_thrust_location = new SceneNode(*scene_manager_, model_node_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.14f, 0.59f)), 90.0f, glm::vec3(0, 0, 1)));
	side_thrust_ = new Propeller(*scene_manager_, side_thrust_location, glm::mat4(1.0f));
	SceneLeafModel* side_thrust_model_leaf = new SceneLeafModel(*side_thrust_, NULL, *propeller_model, *sub_material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	
	SceneNode* forward_thrust_location = new SceneNode(*scene_manager_, model_node_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.2f, 0.02f, -0.36f)), 90.0f, glm::vec3(1, 0, 0)));
	forward_thrust_ = new Propeller(*scene_manager_, forward_thrust_location, glm::mat4(1.0f));
	SceneLeafModel* forward_thrust_model_leaf = new SceneLeafModel(*forward_thrust_, NULL, *propeller_model, *sub_material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	
	SceneNode* forward_thrust_location2 = new SceneNode(*scene_manager_, model_node_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(-0.22f, 0.02f, -0.36f)), 90.0f, glm::vec3(1, 0, 0)));
	forward_thrust2_ = new Propeller(*scene_manager_, forward_thrust_location2, glm::mat4(1.0f));
	SceneLeafModel* forward_thrust_model_leaf2 = new SceneLeafModel(*forward_thrust2_, NULL, *propeller_model, *sub_material, CausticShader::getShader(), false, false, OBJECT, ShadowRenderer::STATIC_SHADOW);
	
	robot_hand_ = new RobotHand(*scene_manager_, model_node_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -0.5f)));
	scene_manager_->addUpdateableEntity(*robot_hand_);
	robot_hand_->unfoldArm(false);
}

AUV::~AUV()
{
	
}

void AUV::setDesiredOrientation(float yaw, float pitch)
{
	desired_yaw_ = yaw + 180;
	if (desired_yaw_ <= -360)
	{
		desired_yaw_ += 360;
	}
	else if (desired_yaw_ >= 360)
	{
		desired_yaw_ -= 360;
	}
	
	desired_pitch_ = pitch;
	face_forward_ = false;
}

void AUV::unsetDesiredOrientation()
{
	desired_yaw_ = 0;
	desired_pitch_ = 0;
	face_forward_ = true;
}

void AUV::setBillBoard(BillBoard& bill_board)
{
	status_label_ = &bill_board;
}

void AUV::setBillBoardUVs(const std::vector<glm::vec2>& uv)
{
	status_label_->setUVMapping(uv);
	status_label_->setVisible(true);
	status_label_->enableBlinking(true);
	bill_board_time_ = 10.0f;
}

void AUV::prepare(float dt)
{
	bill_board_time_ -= dt;
	if (status_label_ != NULL && bill_board_time_ < 0)
	{
		status_label_->setVisible(false);
	}
	//std::cout << name_ << " - (" << getGlobalLocation().x << ", " << getGlobalLocation().x << ", " << getGlobalLocation().x << ")" << std::endl;
	total_time_ += dt;
#ifdef _WIN32
	float begin_prepare = float(GetTickCount()) / 1000.0f;
#endif
#ifdef __linux__
	float begin_prepare = float(clock()) / CLOCKS_PER_SEC;
#endif
	
	// Update the velocity of the AUV.
	float friction = 0.0001f;
	if (velocity_ > 0 && friction * dt < velocity_)
		velocity_ -= friction * dt; // Friction.
	else if (velocity_ < 0 && -friction * dt > velocity_)
		velocity_ += friction * dt; // Friction.
	else
		velocity_ = 0.0f;
	bool received_input = true;
	if (velocity_ < desired_velocity_) velocity_ += 2.0f * dt;
	else if (velocity_ > desired_velocity_) velocity_ -= 2.0f * dt;

	
	if (desired_velocity_ == 0)
	{
		velocity_ *= 0.1;
	}
	
	// Check the orientation of the auv_model.
	glm::vec4 auv_facing4 = glm::vec4(0, 0, -1, 1);
	auv_facing4 = model_node_->getCompleteTransformation() * auv_facing4;
	glm::vec3 auv_facing(auv_facing4.x, auv_facing4.y, auv_facing4.z);
	auv_facing -= model_node_->getGlobalLocation();
	
	float d_yaw = 0;
	
	if (face_forward_)
	{
		d_yaw = atan2(auv_facing.z, auv_facing.x) - atan2(desired_direction_.z, desired_direction_.x);
		d_yaw *= 180.0f / M_PI;
	}
	else
	{
		d_yaw = desired_yaw_ - yaw_;
	}

	if (d_yaw > 180) d_yaw -= 360;
	if (d_yaw < -180) d_yaw += 360;

	float rotation_speed = 20.0f;
	if (d_yaw > 0)
	{
		d_yaw = std::min(d_yaw, dt * rotation_speed);
	}
	else
	{
		d_yaw = std::max(d_yaw, -dt * rotation_speed);
	}

	yaw_ += d_yaw;
	
	glm::mat4 model_location(1.0f);
	model_location = glm::rotate(model_location, yaw_, glm::vec3(0.0f, 1.0f, 0.0f));
	
	model_node_->setTransformation(model_location);
	
	// Move the AUV towards the location were we want to go.
	glm::mat4 org_transformation = local_transformation_;
	
	local_transformation_ = glm::translate(glm::mat4(1.0f), getLocalLocation() + glm::normalize(desired_direction_) * velocity_ * dt);
	updateTransformations();
	
	
	glm::vec3 actual_location = glm::rotate(desired_direction_, yaw_, glm::vec3(0, 1, 0));
	
	// Activate the thrusters as appropriate.
	if (d_yaw != 0)
	{
		side_thrust_->setRotationSpeed(10000 * d_yaw);
	}
	else 
	{
		side_thrust_->setRotationSpeed(1000 * actual_location.x * velocity_);
	}
	
	forward_thrust_->setRotationSpeed((received_input ? 10000 : 1000) * (velocity_ / max_velocity_));
	forward_thrust2_->setRotationSpeed((received_input ? 10000 : 1000) * (velocity_ / max_velocity_));
	
	down_thrust_->setRotationSpeed(1000 * actual_location.y * velocity_);
	down_thrust2_->setRotationSpeed(1000 * actual_location.y * velocity_);

	std::vector<CollisionInfo> collision_infos;

	bool found_collision = false;
	if (current_region_ != NULL && false)
	{
		found_collision = current_region_->getCollisions(*this, collision_infos);
	}
	else
	{
		found_collision = scene_manager_->getRoot().getCollisions(*this, collision_infos);
	}
	
	if (found_collision)
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss_ << "Collision detected. Entities checked: " << nr_entities_checked << "|";
#endif
		std::cout <<"Collision detected!" << std::endl;
		velocity_ = 0.0f;
		local_transformation_ = org_transformation;
		updateTransformations();
	}
	
	SceneNode::prepare(dt);
	auv_waypoint_->position_ = getGlobalLocation();
}

void AUV::onCollision(const CollisionInfo& collision_info)
{
	
}
