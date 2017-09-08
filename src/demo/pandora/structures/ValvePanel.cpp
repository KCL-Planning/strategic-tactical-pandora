#include "ValvePanel.h"

#include "dpengine/entities/behaviours/RotateBehaviour.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/shaders/ToonShader.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shapes/Cube.h"
#include "dpengine/collision/ConvexPolygon.h"

int ValvePanel::global_valve_panel_id_ = 0;

ValvePanel::ValvePanel(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, const std::string& name, DreadedPE::Texture& texture)
	: DreadedPE::Entity(scene_manager, parent, transformation, OBSTACLE, name), is_examined_(false)
{
	std::stringstream ss;
	ss << global_valve_panel_id_;
	id_ = ss.str();
	
	ss.str(std::string());
	ss << "p" << global_valve_panel_id_;
	name_ = ss.str();
	
	++global_valve_panel_id_;
	
	DreadedPE::MaterialLightProperty* ambient = new DreadedPE::MaterialLightProperty(0.1f, 0.1f, 0.1f, 1.0f);
	DreadedPE::MaterialLightProperty* diffuse = new DreadedPE::MaterialLightProperty(0.9f, 0.9f, 0.9f, 1.0f);
	DreadedPE::MaterialLightProperty* specular = new DreadedPE::MaterialLightProperty(0.11f, 0.11f, 0.11f, 1.0f);
	DreadedPE::MaterialLightProperty* panel_colour = new DreadedPE::MaterialLightProperty(1.0f, 1.0f, 0.0f, 1.0f);
	DreadedPE::Material* panel_material = new DreadedPE::Material(*ambient, *diffuse, *specular, *panel_colour);
	panel_material->add2DTexture(texture);
	
	// Create the panel.
	DreadedPE::Cube* panel = new DreadedPE::Cube(1.0f, 2.0f, 0.2f);
	DreadedPE::SceneLeafModel* panel_leaf = new DreadedPE::SceneLeafModel(*this, NULL, *panel, *panel_material, ToonShader::getShader(), false, false);
	
	DreadedPE::ConvexPolygon* panel_bc = new DreadedPE::ConvexPolygon(*this, 1.0f, 2.0f, 0.2f);
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
