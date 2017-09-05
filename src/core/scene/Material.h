#ifndef CORE_SCENE_MATERIAL_H
#define CORE_SCENE_MATERIAL_H

#include <vector>
#include "GL/glew.h"

class Texture;
struct MaterialLightProperty
{
	MaterialLightProperty(float red, float green, float blue, float alpha)
		: red_(red), green_(green), blue_(blue), alpha_(alpha)
	{

	}

	float red_, green_, blue_, alpha_;
};

class Material
{
public:
	Material(MaterialLightProperty& ambient, MaterialLightProperty& diffuse, MaterialLightProperty& specular, MaterialLightProperty& emissive, float transparency = 0.0f);

	void add1DTexture(Texture& texture) { texture_1d_.push_back(&texture); }
	void add2DTexture(Texture& texture) { texture_2d_.push_back(&texture); }
	void addCubeTexture(Texture& texture) { texture_cube_.push_back(&texture); }

	const std::vector<Texture*>& get1DTextures() const { return texture_1d_; }
	const std::vector<Texture*>& get2DTextures() const { return texture_2d_; }
	const std::vector<Texture*>& getCubeTextures() const { return texture_cube_; }
	
	void clear1DTextures() { texture_1d_.clear(); }
	void clear2DTextures() { texture_2d_.clear(); }
	void clearCubeTextures() { texture_cube_.clear(); }

	const MaterialLightProperty& getAmbient() const { return ambient_; }
	const MaterialLightProperty& getDiffuse() const { return diffuse_; }
	const MaterialLightProperty& getSpecular() const { return specular_; }
	const MaterialLightProperty& getEmissive() const { return emissive_; }
	
	MaterialLightProperty& getAmbient() { return ambient_; }
	MaterialLightProperty& getDiffuse() { return diffuse_; }
	MaterialLightProperty& getSpecular() { return specular_; }
	MaterialLightProperty& getEmissive() { return emissive_; }
	
	void setAmbient(MaterialLightProperty& property) { ambient_ = property; }
	void setDiffuse(MaterialLightProperty& property) { diffuse_ = property; }
	void setSpecular(MaterialLightProperty& property) { specular_ = property; }
	void setEmissive(MaterialLightProperty& property) { emissive_ = property; }
	
	float getTransparency() const { return transparency_; }
	void setTransparency(float transparency) { transparency_ = transparency; }
private:

	MaterialLightProperty ambient_;
	MaterialLightProperty diffuse_;
	MaterialLightProperty specular_;
	MaterialLightProperty emissive_;

	std::vector<Texture*> texture_1d_, texture_2d_, texture_cube_;
	
	float transparency_;
};

#endif
