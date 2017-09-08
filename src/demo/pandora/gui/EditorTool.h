#ifndef PANDORA_GUI_EDITOR_TOOL_H
#define PANDORA_GUI_EDITOR_TOOL_H

#include <memory>
#include <glm/glm.hpp>
#include <string>
#include <map>

#include "dpengine/gui/Frame.h"

namespace DreadedPE
{
	class Theme;
	class Font;
	class Label;
	class Button;
	class Camera;
	class HeightMap;
	class SceneManager;
	class SceneLeafModel;
	class Material;
	class Entity;
};

class MissionSite;
class OntologyInterface;
class Structure;

class EditorTool : public DreadedPE::Frame
{
public:
	EditorTool(DreadedPE::SceneManager& scene_manager, OntologyInterface& ontology, DreadedPE::Theme& theme, DreadedPE::Font& font, float x, float y, DreadedPE::Camera& camera, DreadedPE::HeightMap& height_map);
	
	void buttonPressed(const DreadedPE::Button& source);
	
	void update(float dt);
private:
	
	enum EDITOR_MODE { IDLE, REMOVE, ADD_PILLAR, ADD_MANIFOLD, ADD_VALVE_PANEL };
	
	bool getPointOnSeaBed(int mouse_x, int mouse_y, glm::vec3& collision);
	
	void setCurrentMode(EDITOR_MODE mode);
	
	DreadedPE::SceneManager* scene_manager_;
	OntologyInterface* ontology_;
	DreadedPE::Label* selected_entity_label_;
	DreadedPE::Button* submit_button_;
	DreadedPE::Button* remove_button_;
	DreadedPE::Button* add_pillar_button_;
	DreadedPE::Button* add_manifold_button_;
	DreadedPE::Button* add_valve_panel_button_;
	DreadedPE::Camera* camera_;
	DreadedPE::HeightMap* height_map_;
	
	MissionSite* mission_site_;
	
	std::vector<std::pair<DreadedPE::SceneLeafModel*, std::shared_ptr<const DreadedPE::Material> > > org_materials_;
	std::vector<std::shared_ptr<DreadedPE::Material> > tmp_materials_;
	Structure* selected_structure_;
	
	bool submitted_;
	
	EDITOR_MODE current_mode_;
	static std::string status_string_[];
};
#endif
