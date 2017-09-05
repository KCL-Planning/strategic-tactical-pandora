#include "SceneLeaf.h"

#include <vector>
#include <glm/glm.hpp>

class Light;
class Frustum;
class SceneVisitor;
class ShaderInterface;
class SceneNode;
class InFrustumCheck;

class SceneLeafLight : public SceneLeaf
{
public:
	SceneLeafLight(SceneNode& parent, InFrustumCheck* frustum_checker, Light& light);
	virtual ~SceneLeafLight();

	void prepare(float dt);
	void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights);
	void accept(SceneVisitor& visitor) const;

	Light& getLight() const { return *light_; }

	void draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>&, ShaderInterface* shader = NULL) const
	{
	
	}

	void initialiseFrustrumChecker() { }

private:
	Light* light_;
	float total_time_;
};
