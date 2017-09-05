#include "Valve.h"

#include "../../../core/entities/behaviours/RotateBehaviour.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/shaders/ToonShader.h"
#include "../../../core/scene/Material.h"
#include "../../../core/texture/TargaTexture.h"
#include "../../../core/texture/Texture.h"
#include "../../../shapes/Cube.h"
#include "../../../core/collision/BoxCollision.h"

//int Valve::global_valve_id_ = 0;

Valve::Valve(Structure& structure, SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const std::string& name)
	: Entity(scene_manager, parent, transformation, OBSTACLE, name), desired_rotation_(0), structure_(&structure), nr_times_blocked_(0)
{
	Texture* texture = TargaTexture::loadTexture("data/textures/grass.tga");
	//std::stringstream ss;
	//ss << global_valve_id_;
	//id_ = ss.str();
	
	//ss.str(std::string());
	//ss << "v" << global_valve_id_;
	//name_ = ss.str();
	//++global_valve_id_;
	
	MaterialLightProperty* ambient = new MaterialLightProperty(0.1f, 0.1f, 0.1f, 1.0f);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0.9f, 0.9f, 0.9f, 1.0f);
	MaterialLightProperty* specular = new MaterialLightProperty(0.11f, 0.11f, 0.11f, 1.0f);
	MaterialLightProperty* valve_colour = new MaterialLightProperty(0.3f, 0.3f, 0.3f, 1.0f);
	Material* valve_material = new Material(*ambient, *diffuse, *specular, *valve_colour);
	valve_material->add2DTexture(*texture);
	
	// Create the T-valve.
	Cube* bottom_valve = new Cube(0.1f, 0.1f, 0.3f);
	Cube* top_valve = new Cube(0.3f, 0.1f, 0.1f);
	
	SceneNode* valve_node = new SceneNode(scene_manager, this, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.25f)));
	SceneNode* bottom_valve_node = new SceneNode(scene_manager, valve_node, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)));
	SceneNode* top_valve_node = new SceneNode(scene_manager, valve_node, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.15f)));
	SceneLeafModel* bottom_valve_leaf = new SceneLeafModel(*bottom_valve_node, NULL, *bottom_valve, *valve_material, ToonShader::getShader(), false, false);
	SceneLeafModel* top_valve_leaf = new SceneLeafModel(*top_valve_node, NULL, *top_valve, *valve_material, ToonShader::getShader(), false, false);
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
	
	local_transformation_ = glm::rotate(local_transformation_, rotation, glm::vec3(0, 0, 1));
	SceneNode::prepare(dt);
	
	desired_rotation_ -= rotation;
}
