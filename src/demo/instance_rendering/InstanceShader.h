#ifndef DEMO_INSTANCE_RENDERING_INSTANCE_SHADER_H
#define DEMO_INSTANCE_RENDERING_INSTANCE_SHADER_H

#include <glm/glm.hpp>
#include <vector>

#include "../../core/shaders/glslshader.h"
#include "../../core/shaders/ShaderInterface.h"
#include "../../core/shaders/LightShader.h"

class SceneLeafInstanced;

class InstanceShader : public LightShader
{
public:
	virtual void initialise(const SceneLeafInstanced& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);
	
	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{
		
	}
	
	void initialise(const SceneLeafLight& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
	{
		
	}
	
	static InstanceShader& getShader();

protected:
	InstanceShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
	GLuint modelview_matrix_loc_, model_matrix_loc_, projection_matrix_loc_, view_matrix_loc_, texture0_loc_, material_ambient_loc_, material_diffuse_loc_, material_specular_loc_, material_emissive_loc_, transparency_loc_;
	
private:
	static InstanceShader* shader_;
};

#endif
