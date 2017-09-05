#ifndef BOGLGP_TERRAIN_H
#define BOGLGP_TERRAIN_H

#include <glm/glm.hpp>

#ifdef _WIN32
#include <windows.h>
#endif

#include <string>
#include <vector>
#include "GL/glew.h"
#include "Shape.h"

using std::string;
using std::vector;

class Camera;
class LightShader;
class LightManager;
class TargaImage;

class Terrain: public Shape
{
public:
	Terrain();
	bool loadHeightmap(const string& rawFile, int width);
	bool loadHeightmap(const TargaImage& height_map);
	bool createHeightmap(int width, float height);
	bool createRandomHeightmap(int width, float max_height, float min_height);
	//void prepare(float dt) {}
    //void render(const Camera& cam, bool render_shadow, const LightManager& light_manager, GLSLProgram* shader);
	float getHeight(float x, float y) const;

	unsigned int getWidth() const { return width_; }
	const std::vector<float>& getHeights() const { return heights_; }
private:
    void generateVertices(const vector<float> heights, int width);
    void generateIndices(int width);
    void generateTexCoords(int width);
    void generateNormals();

	unsigned int width_;
	vector<float> heights_;
};

#endif
