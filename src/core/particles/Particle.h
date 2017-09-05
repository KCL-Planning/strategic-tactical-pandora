#ifndef CORE_PARTICLES_PARTICLE_H
#define CORE_PARTICLES_PARTICLE_H

#include <glm/glm.hpp>

struct Particle
{
	Particle()
		: position_(glm::vec3(0, 0, 0)), velocity_(glm::vec3(0, 0, 0)), lifetime_(0), type_(0)
	{

	}

	Particle(const glm::vec3& position, const glm::vec3& velocity, float lifetime, float type)
		: position_(position), velocity_(velocity), lifetime_(lifetime), type_(type)
	{

	}

	glm::vec3 position_;
	glm::vec3 velocity_;
	float lifetime_;
	float type_;
};

#endif
