#ifndef DEMO_PANDORA_GUI_WAYPOINT_LABELER_H
#define DEMO_PANDORA_GUI_WAYPOINT_LABELER_H

#include <memory>
#include <vector>
#include <dpengine/scene/SceneNode.h>
#include "../RRTUpdateListener.h"

namespace DreadedPE
{
	class Material;
	class SceneManager;
	class SceneLeafModel;
	class Texture;
};

class RRT;
class TextBanner;

class WaypointLabeler : public DreadedPE::SceneNode, RRTUpdateListener
{
public:
	WaypointLabeler(DreadedPE::SceneManager& scene_manager, RRT& rrt, DreadedPE::SceneNode& parent);
	
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
	std::shared_ptr<DreadedPE::Material> material_;
	std::vector<DreadedPE::SceneLeafModel*> models_;
	DreadedPE::Texture* font_texture_;
};

#endif
