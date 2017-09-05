#ifndef CORE_LOADERS_ASSIMP_LOADER_H
#define CORE_LOADERS_ASSIMP_LOADER_H

#include <string>
#include <map>
#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <assimp/material.h>

class Bone;
class Animation;
class AnimatedShape;
class Texture;
class AnimatedModel;
class Material;
class BoneNode;
struct aiMesh;
struct aiMaterial;
class SceneManager;

class AssimpLoader
{
public:
	static std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > LoadModel(SceneManager& scene_manager, const std::string& filename);
private:
	static bool loadMesh(const aiMesh& mesh, unsigned int vertex_offset, AnimatedModel& animated_model);
	static void loadMaterial(unsigned int material_index, const aiMaterial& material, const std::string& relative_path, std::map<aiTextureType, std::vector<Texture*>* >& texture_mappings);

	//void clear();

	//std::vector<glm::vec3> m_vertices_;
	//std::vector<glm::vec2> m_tex_coords_;
	//std::vector<GLuint> m_indices_;
	//std::vector<glm::vec3> m_normals_;

	//std::vector<Animation*> animations_;
	//std::vector<Bone*> bones_;
	//BoneNode* root_node_;
	
	//std::map<aiTextureType, std::vector<Texture*>* > texture_mappings_;
};

#endif
