#include "Valve.h"

#include <glm/gtc/matrix_transform.hpp>

#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/shaders/ToonShader.h"
#include "dpengine/scene/Material.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/texture/Texture.h"
#include "dpengine/shapes/Cube.h"
#include "dpengine/collision/ConvexPolygon.h"

//int Valve::global_valve_id_ = 0;

Valve::Valve(Structure& structure, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, const std::string& name)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, name), desired_rotation_(0), structure_(&structure), nr_times_blocked_(0)
{
	DreadedPE::Texture* texture = DreadedPE::TargaTexture::loadTexture("data/textures/grass.tga");
	//std::stringstream ss;
	//ss << global_valve_id_;
	//id_ = ss.str();
	
	//ss.str(std::string());
	//ss << "v" << global_valve_id_;
	//name_ = ss.str();
	//++global_valve_id_;
	
	DreadedPE::MaterialLightProperty ambient(0.1f, 0.1f, 0.1f, 1.0f);
	DreadedPE::MaterialLightProperty diffuse(0.9f, 0.9f, 0.9f, 1.0f);
	DreadedPE::MaterialLightProperty specular(0.11f, 0.11f, 0.11f, 1.0f);
	DreadedPE::MaterialLightProperty valve_colour(0.3f, 0.3f, 0.3f, 1.0f);
	std::shared_ptr<DreadedPE::Material> valve_material(std::make_shared<DreadedPE::Material>(ambient, diffuse, specular, valve_colour));
	valve_material->add2DTexture(*texture);
	
	// Create the T-valve.
	std::shared_ptr<DreadedPE::Cube> bottom_valve(std::make_shared<DreadedPE::Cube>(0.1f, 0.1f, 0.3f));
	std::shared_ptr<DreadedPE::Cube> top_valve(std::make_shared<DreadedPE::Cube>(0.3f, 0.1f, 0.1f));
	
	DreadedPE::SceneNode* valve_node = new DreadedPE::SceneNode(scene_manager, this, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.25f)));
	DreadedPE::SceneNode* bottom_valve_node = new DreadedPE::SceneNode(scene_manager, valve_node, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)));
	DreadedPE::SceneNode* top_valve_node = new DreadedPE::SceneNode(scene_manager, valve_node, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.15f)));
	DreadedPE::SceneLeafModel* bottom_valve_leaf = new DreadedPE::SceneLeafModel(*bottom_valve_node, NULL, bottom_valve, valve_material, DreadedPE::ToonShader::getShader(), false, false);
	DreadedPE::SceneLeafModel* top_valve_leaf = new DreadedPE::SceneLeafModel(*top_valve_node, NULL, top_valve, valve_material, DreadedPE::ToonShader::getShader(), false, false);
}

void Valve::prepare(float dt)
{
	float rotation = dt * 10;
	if (desired_rotation_ < 0.0f)
	{
		rotation = -rotation;
		if (desired_rotation_ > rotation)
		{
			rotation = desired_rotation_;
		}
	}
	else
	{
		if (desired_rotation_ < rotation)
		{
			rotation = desired_rotation_;
		}
	}
	
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(rotation), glm::vec3(0, 0, 1));
	DreadedPE::SceneNode::prepare(dt);
	
	desired_rotation_ -= rotation;
}
