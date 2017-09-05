#include "ValvePanel.h"

#include "../../../core/entities/behaviours/RotateBehaviour.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../core/shaders/ToonShader.h"
#include "../../../core/scene/Material.h"
#include "../../../shapes/Cube.h"
#include "../../../core/collision/BoxCollision.h"

int ValvePanel::global_valve_panel_id_ = 0;

ValvePanel::ValvePanel(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const std::string& name, Texture& texture)
	: Entity(scene_manager, parent, transformation, OBSTACLE, name), is_examined_(false)
{
	std::stringstream ss;
	ss << global_valve_panel_id_;
	id_ = ss.str();
	
	ss.str(std::string());
	ss << "p" << global_valve_panel_id_;
	name_ = ss.str();
	
	++global_valve_panel_id_;
	
	MaterialLightProperty* ambient = new MaterialLightProperty(0.1f, 0.1f, 0.1f, 1.0f);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0.9f, 0.9f, 0.9f, 1.0f);
	MaterialLightProperty* specular = new MaterialLightProperty(0.11f, 0.11f, 0.11f, 1.0f);
	MaterialLightProperty* panel_colour = new MaterialLightProperty(1.0f, 1.0f, 0.0f, 1.0f);
	Material* panel_material = new Material(*ambient, *diffuse, *specular, *panel_colour);
	panel_material->add2DTexture(texture);
	
	// Create the panel.
	Cube* panel = new Cube(1.0f, 2.0f, 0.2f);
	SceneLeafModel* panel_leaf = new SceneLeafModel(*this, NULL, *panel, *panel_material, ToonShader::getShader(), false, false);
	
	BoxCollision* panel_bc = new BoxCollision(*this, 1.0f, 2.0f, 0.2f);
	addCollision(*panel_bc);
	
	glm::vec4 visible_point(0, 0, -3, 1);
	
	updateTransformations();
	
	interact_location_ = glm::vec3(getCompleteTransformation() * glm::vec4(0, 0, 2, 1));
}

void ValvePanel::addValve(Valve& valve)
{
	valves_.push_back(&valve);
}

/*
void ValvePanel::prepare(float dt)
{
	Entity::prepare(dt);
}

bool ValvePanel::activate(Entity& activator)
{
	valve_rotate_behaviour_->activate(activator);
}
*/
