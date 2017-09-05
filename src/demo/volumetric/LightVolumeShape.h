#ifndef DEMO_VOLUMETRIC_LIGHT_VOLUME_SHAPE_H
#define DEMO_VOLUMETRIC_LIGHT_VOLUME_SHAPE_H

#include "../../shapes/Shape.h"

class ShadowRenderer;
class PointLight;
class SceneLeafModel;
class SceneManager;

/**
 * Visualises the volume that a light illiminates.
 */
class LightVolumeShape : public Shape
{
public:
	LightVolumeShape(SceneManager& scene_manager, PointLight& point_light);
	
	void setLeafNode(const SceneLeafModel& node) { node_ = &node; }

	//void prepare(float dt);

	//void render();

	const PointLight& getPointLight() const { return *point_light_; }

private:
	const SceneLeafModel* node_;
	PointLight* point_light_;
};

#endif
