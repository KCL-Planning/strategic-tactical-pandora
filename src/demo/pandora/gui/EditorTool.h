#ifndef PANDORA_GUI_EDITOR_TOOL_H
#define PANDORA_GUI_EDITOR_TOOL_H

#include <glm/glm.hpp>
#include <string>
#include <map>

#include "../../../core/gui/Frame.h"

class Theme;
class Font;
class Label;
class Button;
class Camera;
class HeightMap;
class SceneManager;
class MissionSite;
class SceneLeafModel;
class Material;
class Entity;
class OntologyInterface;
class Structure;

class EditorTool : public Frame
{
public:
	EditorTool(SceneManager& scene_manager, OntologyInterface& ontology,Theme& theme, Font& font, float x, float y, Camera& camera, HeightMap& height_map);
	
	void buttonPressed(const Button& source);
	
	void update(float dt);
private:
	
	enum EDITOR_MODE { IDLE, REMOVE, ADD_PILLAR, ADD_MANIFOLD, ADD_VALVE_PANEL };
	
	bool getPointOnSeaBed(int mouse_x, int mouse_y, glm::vec3& collision);
	
	void setCurrentMode(EDITOR_MODE mode);
	
	SceneManager* scene_manager_;
	OntologyInterface* ontology_;
	Label* selected_entity_label_;
	Button* submit_button_;
	Button* remove_button_;
	Button* add_pillar_button_;
	Button* add_manifold_button_;
	Button* add_valve_panel_button_;
	Camera* camera_;
	HeightMap* height_map_;
	
	MissionSite* mission_site_;
	
	std::vector<std::pair<SceneLeafModel*, const Material*> > org_materials_;
	std::vector<Material*> tmp_materials_;
	Structure* selected_structure_;
	
	bool submitted_;
	
	EDITOR_MODE current_mode_;
	static std::string status_string_[];
};
#endif
