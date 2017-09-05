#ifndef CORE_TEXTURE_RANDOM_TEXTURE_H
#define CORE_TEXTURE_RANDOM_TEXTURE_H

#include "Texture.h"

/**
 * Create a texture with random numbers between 0 and 1 that can be used in shaders when
 * there is a need for random numbers.
 */
class RandomTexture : public Texture
{
public:
	RandomTexture(unsigned int size);
private:
	unsigned int size_;
};

#endif
