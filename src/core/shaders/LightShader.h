#ifndef LIGHT_SHADER_H
#define LIGHT_SHADER_H

#include "glslshader.h"
#include "ShaderInterface.h"

class LightManager;

class LightShader : public GLSLProgram, public ShaderInterface
{
public:
	LightShader(const string& vertexShader, const string& fragmentShader);
	LightShader(const string& vertexShader, const string& geometryShader, const string& fragmentShader);

	void resolveUniformNames();

	unsigned int getTypeId(unsigned int light_index) const { return type_id_[light_index]; }
	unsigned int getShadowMatrixId(unsigned int light_index) const { return shadow_matrix_id_[light_index]; }
	unsigned int getEnabledId(unsigned int light_index) const { return enabled_id_[light_index]; }
	unsigned int getAmbientId(unsigned int light_index) const { return ambient_id_[light_index]; }
	unsigned int getDiffuseId(unsigned int light_index) const { return diffuse_id_[light_index]; }
	unsigned int getSpecularId(unsigned int light_index) const { return specular_id_[light_index]; }
	unsigned int getConstantAttenuationId(unsigned int light_index) const { return constant_attenuation_id_[light_index]; }
	unsigned int getLinearAttenuationId(unsigned int light_index) const { return linear_attenuation_id_[light_index]; }
	unsigned int getQuadraticAttenuationId(unsigned int light_index) const { return quadratic_attenuation_id_[light_index]; }
	unsigned int getDirectionId(unsigned int light_index) const { return direction_id_[light_index]; }
	unsigned int getLightAngleId(unsigned int light_index) const { return light_angle_id_[light_index]; }
	unsigned int getPositionId(unsigned int light_index) const { return position_id_[light_index]; }
	unsigned int getDepthTextureId(unsigned int light_index) const { return depth_texture_id_[light_index]; }
	unsigned int getTransparentTextureId(unsigned int light_index) const { return transparent_texture_id_[light_index]; }

	virtual void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);

	static const unsigned int MAX_LIGHTS_ = 5;
	
private:
	unsigned int type_id_[LightShader::MAX_LIGHTS_];
	unsigned int shadow_matrix_id_[LightShader::MAX_LIGHTS_];
	unsigned int enabled_id_[LightShader::MAX_LIGHTS_];
	unsigned int ambient_id_[LightShader::MAX_LIGHTS_];
	unsigned int diffuse_id_[LightShader::MAX_LIGHTS_];
	unsigned int specular_id_[LightShader::MAX_LIGHTS_];
	unsigned int constant_attenuation_id_[LightShader::MAX_LIGHTS_];
	unsigned int linear_attenuation_id_[LightShader::MAX_LIGHTS_];
	unsigned int quadratic_attenuation_id_[LightShader::MAX_LIGHTS_];
	unsigned int direction_id_[LightShader::MAX_LIGHTS_];
	unsigned int light_angle_id_[LightShader::MAX_LIGHTS_];
	unsigned int position_id_[LightShader::MAX_LIGHTS_];
	unsigned int depth_texture_id_[LightShader::MAX_LIGHTS_];
	unsigned int transparent_texture_id_[LightShader::MAX_LIGHTS_];
};

#endif
