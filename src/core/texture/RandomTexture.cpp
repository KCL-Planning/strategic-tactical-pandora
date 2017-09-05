#include "RandomTexture.h"

#include <time.h>
#include <stdlib.h>

RandomTexture::RandomTexture(unsigned int size)
	: Texture(GL_TEXTURE_1D), size_(size)
{
	std::vector<float> random_data;

	srand(time(NULL));
	for (unsigned int i = 0 ; i < size; i++)
	{
		random_data.push_back((float)rand() / (float)RAND_MAX);
		random_data.push_back((float)rand() / (float)RAND_MAX);
		random_data.push_back((float)rand() / (float)RAND_MAX);
	}

	glBindTexture(GL_TEXTURE_1D, texture_id_);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, size, 0.0f, GL_RGB, GL_FLOAT, &random_data[0]);
	glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_REPEAT);
}
