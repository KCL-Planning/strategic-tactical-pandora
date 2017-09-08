#ifndef CORE_SHAPE_SKY_BOX_H
#define CORE_SHAPE_SKY_BOX_H

#include "GL/glew.h"

#include <glm/glm.hpp>

#include "SceneLeafModel.h"

class Cube;
class SceneNode;
class ShaderInterface;
class Material;

class SkyBoxLeaf : public SceneLeafModel
{
public:
	SkyBoxLeaf(SceneNode& parent, Shape& shape, ShaderInterface& shader, Material& material);//, const std::string& left, const std::string& right, const std::string& top, const std::string& bottom, const std::string& back, const std::string& front);

	virtual void draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader = NULL) const;

	virtual void prepare(float dt);
	virtual void preRender(const Frustum& frustum, Renderer& renderer);

	virtual void accept(Renderer& renderer) const;

private:
	//Cube* inverted_box_;
	//GLuint cube_texture_id_;
};

#endif
