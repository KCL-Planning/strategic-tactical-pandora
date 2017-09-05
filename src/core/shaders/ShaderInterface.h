#ifndef CORE_SHADERS_SHADER_INTERFACE_H
#define CORE_SHADERS_SHADER_INTERFACE_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <vector>

class LightManager;
class SceneLeafLight;
class SceneLeafModel;

/**
 * Each node that can be rendered has a shader attached to it. When the node is rendered then the
 * shader instance is responsible to initialise the shader variables. This allows us to render 
 * parts of the scene differently depending on which shader we use without having to alter the
 * scene nodes or renderer.
 */
class ShaderInterface
{
public:

	/**
	 * Initialise all the uniform and attributes of this shader.
	 * @param light_node The scene node which represents a light.
	 * @param view_matrix The view matrix.
	 * @param model_matrix The model matrix.
	 * @param projection_matrix The projectiong matrix.
	 */
	virtual void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights) = 0;

	/**
	 * Initialise all the uniform and attributes of this shader.
	 * @param model_node The scene node which represents a model.
	 * @param view_matrix The view matrix.
	 * @param model_matrix The model matrix.
	 * @param projection_matrix The projectiong matrix.
	 */
	virtual void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights) = 0;
};

#endif
