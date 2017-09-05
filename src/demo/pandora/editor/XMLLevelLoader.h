#ifndef PANDORA_EDITOR_XML_LEVEL_EDITOR_LOADER_H
#define PANDORA_EDITOR_XML_LEVEL_EDITOR_LOADER_H

#include <map>
#include <vector>
#include <string>
#include "../3rdparty/pugixml/pugixml.hpp"
#include "../ontology/Pose.h"

class SceneManager;
class SceneNode;
class Ontology;
class HeightMap;

struct StructureDescription
{
	std::string name_;
	std::string file_name_;
	std::string texture_;
	bool can_recharge_;
	std::vector<Pose> inspection_points_;
};

std::ostream& operator<<(std::ostream& os, const StructureDescription& sd);

class XMLLevelLoader
{
public:
	XMLLevelLoader(SceneManager& scene_manager, SceneNode& root, Ontology& ontology, HeightMap& height_map);
	void loadLevel(const std::string& xml_file);
	
	const glm::mat4& getAUVLocation() const { return auv_location_; }
private:
	
	void loadStructures(const pugi::xml_node& node);
	void loadWorld(const pugi::xml_node& node, bool separate_inspection_goals);
	
	void printXMLNode(const pugi::xml_node& node, unsigned int depth = 0) const;
	
	SceneManager* scene_manager_;
	SceneNode* root_;
	Ontology* ontology_;
	HeightMap* height_map_;
	
	glm::mat4 auv_location_;
	
	std::map<std::string, StructureDescription> structure_mapping_;
};

#endif
