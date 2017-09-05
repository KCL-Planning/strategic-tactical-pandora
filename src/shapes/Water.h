#ifndef WATER_H
#define WATER_H

#include <GL/glew.h>
#include <glm/glm.hpp>

#include <vector>
#include "../core/shaders/glslshader.h"
#include "Shape.h"

class Camera;
class LightManager;

class Water: public Shape
{
public:
	Water(unsigned int width, unsigned int height, float depth);
	void progressWaves(float dt);

	const std::vector<float> getWaveHeights() const { return m_wave_effect_; }
	GLuint getWaveHeightsBufferId() const { return wave_effect_index_; }
	unsigned int getHeight() const { return height_; }
	unsigned int getWidth() const { return width_; }

private:
	unsigned int width_, height_;

	GLuint wave_effect_index_;
	std::vector<float> m_wave_effect_;
	float depth_, total_time_;
};

#endif
