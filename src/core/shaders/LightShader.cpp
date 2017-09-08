#include <glm/gtc/type_ptr.hpp>

#include "dpengine/shaders/LightShader.h"
#include "dpengine/light/Light.h"
#include "dpengine/scene/SceneLeafLight.h"
#include "dpengine/renderer/ShadowRenderer.h"
#include "dpengine/texture/Texture.h"

namespace DreadedPE
{

LightShader::LightShader(const string& vertexShader, const string& fragmentShader)
	: GLSLProgram(vertexShader, fragmentShader)
{

}

LightShader::LightShader(const string& vertexShader, const string& geometryShader, const string& fragmentShader)
	: GLSLProgram(vertexShader, geometryShader, fragmentShader)
{

}

void LightShader::prepareToRender(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
	//bindShader();
	for (unsigned int light_nr = 0; light_nr < lights.size() && light_nr < MAX_LIGHTS_; ++light_nr)
	{
		const SceneLeafLight* light = lights[light_nr];

		glUniformMatrix4fv(getShadowMatrixId(light_nr), 1, false, glm::value_ptr(light->getLight().getShadowMatrix()));
		glUniform1i(getEnabledId(light_nr), true);
		glUniform1i(getTypeId(light_nr), light->getLight().getType());
		glUniform4f(getAmbientId(light_nr), light->getLight().getAmbient().r, light->getLight().getAmbient().g, light->getLight().getAmbient().b, 1.0f);
		glUniform4f(getDiffuseId(light_nr), light->getLight().getDiffuse().r, light->getLight().getDiffuse().g, light->getLight().getDiffuse().b, 1.0f);
		glUniform4f(getSpecularId(light_nr),light->getLight().getSpecular().r, light->getLight().getSpecular().g, light->getLight().getSpecular().b, 1.0f);
		glUniform1f(getConstantAttenuationId(light_nr), light->getLight().getConstantAttenuation());
		glUniform1f(getLinearAttenuationId(light_nr), light->getLight().getLinearAttenuation());
		glUniform1f(getQuadraticAttenuationId(light_nr), light->getLight().getQuadraticAttentuation());
		glUniform3f(getDirectionId(light_nr), light->getLight().getDirection()[0], light->getLight().getDirection()[1], light->getLight().getDirection()[2]);
		glUniform1f(getLightAngleId(light_nr), light->getLight().getAngle());
		glUniform4f(getPositionId(light_nr), light->getLight().getLocation()[0], light->getLight().getLocation()[1], light->getLight().getLocation()[2], 1.0f);
		glUniform1i(getDepthTextureId(light_nr), light->getLight().getShadowRendererTextureID());
	}

	for (unsigned int light_nr = lights.size(); light_nr < MAX_LIGHTS_; ++light_nr)
	{
		glUniformMatrix4fv(getShadowMatrixId(light_nr), 1, false, glm::value_ptr(glm::mat4(1.0)));
		glUniform1i(getEnabledId(light_nr), false);
		glUniform1i(getTypeId(light_nr), 0);
		glUniform4f(getAmbientId(light_nr), 0.0f, 0.0f, 0.0f, 1.0f);
		glUniform4f(getDiffuseId(light_nr), 0.0f, 0.0f, 0.0f, 1.0f);
		glUniform4f(getSpecularId(light_nr), 0.0f, 0.0f, 0.0f, 1.0f);
		glUniform1f(getConstantAttenuationId(light_nr), 0.0f);
		glUniform1f(getLinearAttenuationId(light_nr), 0.0f);
		glUniform1f(getQuadraticAttenuationId(light_nr), 0.0f);
		glUniform3f(getDirectionId(light_nr), 0.0f, 0.0f, 0.0f);
		glUniform1f(getLightAngleId(light_nr), 0.0f);
		glUniform4f(getPositionId(light_nr), 0.0f, 0.0f, 0.0f, 1.0f);
		glUniform1i(getDepthTextureId(light_nr), 0);
	}
}

void LightShader::resolveUniformNames()
{
	GLSLProgram::linkProgram();
	for (unsigned int light_nr = 0; light_nr < LightShader::MAX_LIGHTS_; ++light_nr)
	{
		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].type";
			type_id_[light_nr] = getUniformLocation(ss.str());
		}

		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].shadow_matrix";
			shadow_matrix_id_[light_nr] = getUniformLocation(ss.str());
		}

		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].enabled";
			enabled_id_[light_nr] = getUniformLocation(ss.str());
		}

		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].ambient";
			ambient_id_[light_nr] = getUniformLocation(ss.str());
		}

		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].diffuse";
			diffuse_id_[light_nr] = getUniformLocation(ss.str());
		}

		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].specular";
			specular_id_[light_nr] = getUniformLocation(ss.str());
		}
		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].constant_attenuation";
			constant_attenuation_id_[light_nr] = getUniformLocation(ss.str());
		}
		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].linear_attenuation";
			linear_attenuation_id_[light_nr] = getUniformLocation(ss.str());
		}
		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].quadratic_attenuation";
			quadratic_attenuation_id_[light_nr] = getUniformLocation(ss.str());
		}
		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].direction";
			direction_id_[light_nr] = getUniformLocation(ss.str());
		}
		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].light_angle";
			light_angle_id_[light_nr] = getUniformLocation(ss.str());
		}
		{
			std::stringstream ss;
			ss << "lights[" << light_nr << "].position";
			position_id_[light_nr] = getUniformLocation(ss.str());
		}
		{
			std::stringstream ss;
			ss << "depth_texture[" << light_nr << "]";
			depth_texture_id_[light_nr] = getUniformLocation(ss.str());
		}
	}
}

};
