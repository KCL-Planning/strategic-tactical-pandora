#ifndef DEMO_PANDORA_GUI_WAYPOINT_LABELER_H
#define DEMO_PANDORA_GUI_WAYPOINT_LABELER_H

#include <vector>
#include "../../../core/scene/SceneNode.h"
#include "../RRTUpdateListener.h"

class Material;
class RRT;
class SceneManager;
class SceneLeafModel;
class TextBanner;
class Texture;

class WaypointLabeler : public SceneNode, RRTUpdateListener
{
public:
	WaypointLabeler(SceneManager& scene_manager, RRT& rrt, SceneNode& parent);
	
	/**
	 * When the RRT is invalidated we remove all the scene leafs.
	 */
	void rrtInvalidated();
	
	/**
	 * When a new RRT is created we add all the new scene leafs.
	 */
	void rrtUpdated();
	
private:
	const RRT* rrt_;
	Material* material_;
	std::vector<SceneLeafModel*> models_;
	std::vector<TextBanner*> shapes_;
	Texture* font_texture_;
};

#endif
