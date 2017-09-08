#ifndef DEMO_VOLUMETRIC_LIGHT_VOLUME_SHAPE_H
#define DEMO_VOLUMETRIC_LIGHT_VOLUME_SHAPE_H

#include <dpengine/shapes/Shape.h>

namespace DreadedPE
{
class ShadowRenderer;
class SpotLight;
class SceneLeafModel;
class SceneManager;
}

/**
 * Visualises the volume that a light illiminates.
 */
class LightVolumeShape : public DreadedPE::Shape
{
public:
	LightVolumeShape(DreadedPE::SceneManager& scene_manager, DreadedPE::SpotLight& point_light);
	
	void setLeafNode(const DreadedPE::SceneLeafModel& node) { node_ = &node; }

	const DreadedPE::SpotLight& getPointLight() const { return *point_light_; }

private:
	const DreadedPE::SceneLeafModel* node_;
	DreadedPE::SpotLight* point_light_;
};

#endif
