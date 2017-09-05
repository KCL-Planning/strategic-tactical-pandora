#ifndef LIGHT_H
#define LIGHT_H

#include <vector>
#include <glm/glm.hpp>

#include "../shaders/LightShader.h"
#include "../renderer/FrustumCaster.h"

class LightManager;
class GLSLProgram;
class ShadowRenderer;

class Light : public FrustumCaster
{
public:

	enum LIGHT_TYPE { DIRECTIONAL = 0, SPOTLIGHT, POINTLIGHT };

	Light(LIGHT_TYPE type, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float constant_attenuation, float linear_attenuation, float quadratic_attenuation, float close_plane, float far_plane, float light_angle = std::numeric_limits<float>::max(), GLuint shadow_map_dimension = 2048);
	//virtual void initialise(unsigned int texture_id) = 0;
	//virtual void unload() = 0;

	/**
	 * Prepare the shadow map for rendering.
	 */
	virtual void prepare(float dt);
	virtual void preRender(const glm::mat4& transition);

	const glm::mat4& getShadowMatrix() const { return shadow_matrix_; }

	void setId(unsigned int id) { id_ = id; }
	unsigned int getId() const { return id_; }

	//const glm::vec3 getLocation() const { return location_; }
	const glm::vec3& getDirection() const { return direction_; }
	float getYaw() const { return yaw_; }
	float getPitch() const { return pitch_; }
	float getRoll() const { return roll_; }
	float getAngle() const { return light_angle_; }
	LIGHT_TYPE getType() const { return type_; }

	const glm::vec3& getAmbient() const { return ambient_; }
	const glm::vec3& getDiffuse() const { return diffuse_; }
	const glm::vec3& getSpecular() const { return specular_; }
	float getConstantAttenuation() const { return constant_attenuation_; }
	float getLinearAttenuation() const { return linear_attenuation_; }
	float getQuadraticAttentuation() const { return quadratic_attenuation_; }
	float getClosePlane() const { return close_plane_; }
	float getFarPlane() const { return far_plane_; }

	void setAmbient(const glm::vec3& amb) { ambient_ = amb; }

	virtual glm::vec3 getLocation() const { return location_; };
	virtual glm::mat4 getViewMatrix() const;

	GLuint getShadowMapDimension() const { return shadow_map_dimension_; }

	virtual ShadowRenderer& getShadowRenderer() const = 0;

protected:
	LIGHT_TYPE type_;

	glm::vec3 direction_;
	glm::mat4 shadow_matrix_;
	glm::vec3 location_;
	float yaw_, pitch_, roll_;

	glm::vec3 ambient_;
	glm::vec3 diffuse_;
	glm::vec3 specular_;
	float constant_attenuation_, linear_attenuation_, quadratic_attenuation_;
	float light_angle_;
	float close_plane_, far_plane_;
	GLuint shadow_map_dimension_;
	

private:
	unsigned int id_;
	
};
#endif
