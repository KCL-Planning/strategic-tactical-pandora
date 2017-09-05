#include "AssimpLoader.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <sstream>
#include <iostream>

#include <vector>
#include <glm/glm.hpp>
#include <GL/glew.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "targa.h"
#include "../texture/Texture.h"
//#include "../texture/TargaTexture.h"
#include "../texture/FreeImageLoader.h"
#include "../models/AnimatedModel.h"
#include "../scene/Material.h"

#include "../models/Bone.h"
#include "../models/BoneNode.h"
#include "../models/Animation.h"
#include "../models/AnimationNode.h"

#define ASSIMP_LOADER_DEBUG

std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > AssimpLoader::LoadModel(SceneManager& scene_manager, const std::string& filename)
{
	//clear();
#ifdef ASSIMP_LOADER_DEBUG
	std::cout << "Load ASSIMP model: " << filename << "." << std::endl;
#ifdef _WIN32
	std::stringstream ss;
	ss << "Load ASSIMP model: " << filename << "." << std::endl;
	OutputDebugString(ss.str().c_str());
#endif
#endif
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_GenSmoothNormals);
	if (!scene)
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "File does not exist " << filename;
		ss << importer.GetErrorString(); 
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
		std::cout << "File does not exit " << filename << "(" << importer.GetErrorString() << ")" << std::endl;
#endif
		return std::make_pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* >(NULL, NULL);
	}
	
	std::map<aiTextureType, std::vector<Texture*>* >* texture_mappings = new std::map<aiTextureType, std::vector<Texture*>* >();
	AnimatedModel* animated_model = new AnimatedModel();

	if (scene->HasMaterials())
	{
		size_t last_index = filename.find_last_of("/");
		if (last_index == std::string::npos)
		{
			last_index = filename.find_last_of("\\");
		}

		std::string relative_path;
		if (last_index != std::string::npos)
		{
			relative_path = filename.substr(0, last_index);
		}

		for (unsigned int i = 0; i < scene->mNumMaterials; ++i)
		{
			aiMaterial* material = scene->mMaterials[i];
			loadMaterial(i, *material, relative_path, *texture_mappings);
		}
	}
	else
	{
		std::cout << "Scene has no materials: " << filename << "." << std::endl;
		delete texture_mappings;
		delete animated_model;
		return std::make_pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* >(NULL, NULL);
	}

	std::vector<unsigned int> mesh_vertex_offsets;
	unsigned int vertex_offset = 0;

	if (scene->HasMeshes())
	{
		//assert (scene->mNumMeshes == 1);
		for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
		{
			mesh_vertex_offsets.push_back(vertex_offset);
			aiMesh* mesh = scene->mMeshes[i];
			if (!loadMesh(*mesh, vertex_offset, *animated_model))
			{
				continue;
			}
			vertex_offset += mesh->mNumVertices;
		}
	}
	else
	{
		delete texture_mappings;
		delete animated_model;
		std::cout << "Scene has no materials: " << filename << "." << std::endl;
		return std::make_pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* >(NULL, NULL);
	}

	if (scene->HasAnimations())
	{
		//std::ofstream bones_output;
		//bones_output.open("bones.txt");
		std::map<std::string, Bone*> name_to_bone_mapping;
		for (unsigned int j = 0; j < scene->mNumMeshes; ++j)
		{
			for (unsigned int i = 0; i < scene->mMeshes[j]->mNumBones; ++i)
			{
				aiBone* ai_bone = scene->mMeshes[j]->mBones[i];	
				Bone* bone = NULL;

				//bones_output << j << ": " << ai_bone->mName.C_Str() << "." << std::endl;
				if (name_to_bone_mapping.find(ai_bone->mName.C_Str()) == name_to_bone_mapping.end())
				{
					bone = new Bone(ai_bone->mName.C_Str(), 
										  glm::transpose(glm::mat4(ai_bone->mOffsetMatrix[0][0], ai_bone->mOffsetMatrix[0][1], ai_bone->mOffsetMatrix[0][2], ai_bone->mOffsetMatrix[0][3],
													ai_bone->mOffsetMatrix[1][0], ai_bone->mOffsetMatrix[1][1], ai_bone->mOffsetMatrix[1][2], ai_bone->mOffsetMatrix[1][3],
													ai_bone->mOffsetMatrix[2][0], ai_bone->mOffsetMatrix[2][1], ai_bone->mOffsetMatrix[2][2], ai_bone->mOffsetMatrix[2][3],
													ai_bone->mOffsetMatrix[3][0], ai_bone->mOffsetMatrix[3][1], ai_bone->mOffsetMatrix[3][2], ai_bone->mOffsetMatrix[3][3])));
					name_to_bone_mapping[ai_bone->mName.C_Str()] = bone;
					animated_model->getBones().push_back(bone);
				}
				else
				{
					bone = name_to_bone_mapping[ai_bone->mName.C_Str()];
				}

				for (unsigned int k = 0; k < ai_bone->mNumWeights; ++k)
				{
					const aiVertexWeight& vertex_weight = ai_bone->mWeights[k];
					bone->addVertexWeight(vertex_weight.mVertexId + mesh_vertex_offsets[j], vertex_weight.mWeight);
				}
			}
		}

		// Process all the nodes.
		Bone* root_bone = NULL;
		if (name_to_bone_mapping.find(scene->mRootNode->mName.C_Str()) != name_to_bone_mapping.end())
		{
			root_bone = name_to_bone_mapping[scene->mRootNode->mName.C_Str()];
		}

		BoneNode* root_node = new BoneNode(scene_manager, scene->mRootNode->mName.C_Str(), glm::transpose(glm::mat4(scene->mRootNode->mTransformation[0][0], scene->mRootNode->mTransformation[0][1], scene->mRootNode->mTransformation[0][2], scene->mRootNode->mTransformation[0][3], 
																				scene->mRootNode->mTransformation[1][0], scene->mRootNode->mTransformation[1][1], scene->mRootNode->mTransformation[1][2], scene->mRootNode->mTransformation[1][3],
																				scene->mRootNode->mTransformation[2][0], scene->mRootNode->mTransformation[2][1], scene->mRootNode->mTransformation[2][2], scene->mRootNode->mTransformation[2][3],
																				scene->mRootNode->mTransformation[3][0], scene->mRootNode->mTransformation[3][1], scene->mRootNode->mTransformation[3][2], scene->mRootNode->mTransformation[3][3])), NULL, root_bone);
		animated_model->setRootNode(*root_node);

		std::vector<std::pair<BoneNode*, aiNode*> > open_list;
		open_list.push_back(std::make_pair(root_node, scene->mRootNode));
		
#ifdef ASSIMP_LOADER_DEBUG
		std::cout << "Animations: " << scene->mNumAnimations << "." << std::endl;
#ifdef _WIN32
		std::stringstream ss;
		ss << "Animations: " << scene->mNumAnimations << "." << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
#endif
		std::map<std::string, BoneNode*> name_to_node;
		unsigned int nr_nodes = 0;
		while (!open_list.empty())
		{
			++nr_nodes;
			std::pair<BoneNode*, aiNode*> node = open_list[open_list.size() - 1];
			open_list.erase(open_list.end() - 1);

			name_to_node[node.second->mName.C_Str()] = node.first;
			//ss2 << node.second->mName.C_Str() << std::endl;

			for (unsigned int i = 0; i < node.second->mNumChildren; ++i)
			{
				aiNode* child = node.second->mChildren[i];

				Bone* child_bone = NULL;
				if (name_to_bone_mapping.find(child->mName.C_Str()) != name_to_bone_mapping.end())
				{
					child_bone = name_to_bone_mapping[child->mName.C_Str()];
				}
				BoneNode* animation_node = new BoneNode(scene_manager, child->mName.C_Str(), glm::transpose(glm::mat4(child->mTransformation[0][0], child->mTransformation[0][1], child->mTransformation[0][2], child->mTransformation[0][3], 
																						child->mTransformation[1][0], child->mTransformation[1][1], child->mTransformation[1][2], child->mTransformation[1][3],
																						child->mTransformation[2][0], child->mTransformation[2][1], child->mTransformation[2][2], child->mTransformation[2][3],
																						child->mTransformation[3][0], child->mTransformation[3][1], child->mTransformation[3][2], child->mTransformation[3][3])), node.first, child_bone);
				open_list.push_back(std::make_pair(animation_node, child));
			}
		}
		
		for (unsigned int i = 0; i < scene->mNumAnimations; ++i)
		{
			const aiAnimation* ai_animation = scene->mAnimations[i];
			Animation* animation = new Animation(ai_animation->mName.C_Str(), ai_animation->mDuration, ai_animation->mTicksPerSecond);
			
			for (unsigned int j = 0; j < ai_animation->mNumChannels; ++j)
			{
				aiNodeAnim* node_amin = ai_animation->mChannels[j];

				BoneNode* node = name_to_node[node_amin->mNodeName.C_Str()];
				assert (node != NULL);

				std::vector<std::pair<glm::vec3, float> > translate_times;
				for (unsigned int i = 0; i < node_amin->mNumPositionKeys; ++i)
				{
					float translate_time = node_amin->mPositionKeys[i].mTime;
					glm::vec3 translate = glm::vec3(node_amin->mPositionKeys[i].mValue.x, node_amin->mPositionKeys[i].mValue.y, node_amin->mPositionKeys[i].mValue.z);
					translate_times.push_back(std::make_pair(translate, translate_time));
				}

				std::vector<std::pair<glm::fquat, float> > rotate_times;
				for (unsigned int i = 0; i < node_amin->mNumRotationKeys; ++i)
				{
					float rotate_time = node_amin->mRotationKeys[i].mTime;
					glm::fquat rotate = glm::fquat(node_amin->mRotationKeys[i].mValue.w, node_amin->mRotationKeys[i].mValue.x, node_amin->mRotationKeys[i].mValue.y, node_amin->mRotationKeys[i].mValue.z);
					rotate_times.push_back(std::make_pair(rotate, rotate_time));
				}

				std::vector<std::pair<glm::vec3, float> > scale_times;
				for (unsigned int i = 0; i < node_amin->mNumScalingKeys; ++i)
				{
					float scale_time = node_amin->mScalingKeys[i].mTime;
					glm::vec3 scale = glm::vec3(node_amin->mScalingKeys[i].mValue.x, node_amin->mScalingKeys[i].mValue.y, node_amin->mScalingKeys[i].mValue.z);
					scale_times.push_back(std::make_pair(scale, scale_time));
				}

				AnimationNode* animation_node = new AnimationNode(*animation, *node, translate_times, rotate_times, scale_times);
				animation->addAnimationNode(*animation_node);
			}
			animated_model->getAnimations().push_back(animation);
		}
	}

	//AnimatedModel* animated_model = new AnimatedModel(m_vertices_, m_tex_coords_, m_indices_, m_normals_, *root_node_, animations_, bones_);
	animated_model->finalise();
#ifdef ASSIMP_LOADER_DEBUG
	std::cout << "Animations: " << animated_model->getAnimations().size() << "; Bones: " << animated_model->getBones().size() << "; Vertices: " << animated_model->getVertices().size() << "; Texture Coordinates: " << animated_model->getTexCoords().size() << std::endl;
#ifdef _WIN32
		std::stringstream ss2;
		ss2 << "Animations: " << animated_model->getAnimations().size() << "; Bones: " << animated_model->getBones().size() << std::endl;
		OutputDebugString(ss2.str().c_str());
#endif
#endif
	return std::make_pair(animated_model, texture_mappings);
}

bool AssimpLoader::loadMesh(const aiMesh& mesh, unsigned int vertex_offset, AnimatedModel& animated_model)
{
	if (!mesh.HasTextureCoords(0) || mesh.mNumUVComponents[0] != 2)
	{
		return false;
	}

	assert (mesh.HasTextureCoords(0));
	assert (mesh.mNumUVComponents[0] == 2);
	
	// Load the faces, normals, and UV mappings.
	for (unsigned int i = 0; i < mesh.mNumVertices; ++i)
	{
		animated_model.getVertices().push_back(glm::vec3(mesh.mVertices[i].x, mesh.mVertices[i].y, mesh.mVertices[i].z));
		animated_model.getNormals().push_back(glm::vec3(mesh.mNormals[i].x, mesh.mNormals[i].y, mesh.mNormals[i].z));
		animated_model.getTexCoords().push_back(glm::vec2(mesh.mTextureCoords[0][i].x, mesh.mTextureCoords[0][i].y));
	}

	for (unsigned int i = 0; i < mesh.mNumFaces; ++i)
	{
		// We only deal with triangles.
		//assert (mesh.mFaces[i].mNumIndices == 3);
		if (mesh.mFaces[i].mNumIndices != 3) continue;
		animated_model.getIndices().push_back(mesh.mFaces[i].mIndices[0] + vertex_offset);
		animated_model.getIndices().push_back(mesh.mFaces[i].mIndices[1] + vertex_offset);
		animated_model.getIndices().push_back(mesh.mFaces[i].mIndices[2] + vertex_offset);
	}
	
	return true;
}

void AssimpLoader::loadMaterial(unsigned int material_index, const aiMaterial& material, const std::string& relative_path, std::map<aiTextureType, std::vector<Texture*>* >& texture_mappings)
{
	const std::string texture_type_strings[] = { 
		"aiTextureType_NONE", 
		"aiTextureType_DIFFUSE",
		"aiTextureType_SPECULAR",
		"aiTextureType_AMBIENT",
		"aiTextureType_EMISSIVE",
		"aiTextureType_HEIGHT",
		"aiTextureType_NORMALS",
		"aiTextureType_SHININESS",
		"aiTextureType_OPACITY",
		"aiTextureType_DISPLACEMENT",
		"aiTextureType_LIGHTMAP",
		"aiTextureType_REFLECTION",
		"aiTextureType_UNKNOWN"
	};
	
	for (int j = 0; j < aiTextureType_UNKNOWN; ++j)
	{
		std::cout << "Process the texture type: " << texture_type_strings[j] << std::endl;
		aiTextureType texture_type = static_cast<aiTextureType>(j);
		for (unsigned int i = 0; i < material.GetTextureCount(texture_type); ++i)
		{
			aiString path;
			if (material.GetTexture(texture_type, i, &path) == AI_SUCCESS)
			{
				std::string full_path = path.data;

#ifdef ASSIMP_LOADER_DEBUG
#ifdef _WIN32
				std::stringstream ss;
				ss << "Load Texture: " << relative_path << "/" << full_path << "." << std::endl;
				OutputDebugString(ss.str().c_str());
#else
				std::cout << "Load Texture: " << relative_path << "/" << full_path << "." << std::endl;
#endif
#endif
				//Texture* texture = TargaTexture::loadTexture(relative_path + "/" + full_path);
				Texture* texture = FreeImageLoader::loadTexture(relative_path + "/" + full_path);
				if (texture == NULL)
				{
#ifdef _WIN32
					std::stringstream ss;
					ss << "Texture not complete :((";
					MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
					std::cout << "texture not complete :(" << std::endl;
#endif
					return;
				}
				
				std::vector<Texture*>* loaded_textures = NULL;
				if (texture_mappings.find(texture_type) == texture_mappings.end())
				{
					loaded_textures = new std::vector<Texture*>();
					texture_mappings[texture_type] = loaded_textures;
				}
				else
				{
					loaded_textures = texture_mappings[texture_type];
				}
				loaded_textures->push_back(texture);
			}
			else
			{
#ifdef _WIN32
				std::stringstream ss;
				ss << "Could not load texture.";
				MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
				std::cout << "Could not load texture." << std::endl;
#endif
			}
		}
		
		if (material.GetTextureCount(texture_type) == 0)
		{
			//Texture* texture = TargaTexture::loadTexture("data/models/snake_pattern.tga");
			Texture* texture = FreeImageLoader::loadTexture("data/models/snake_pattern.tga");
			if (texture == NULL)
			{
#ifdef _WIN32
				std::stringstream ss;
				ss << "Texture not complete :((";
				MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
				std::cout << "Texture not complete :((" << std::endl;
#endif
				return;
			}

			std::vector<Texture*>* loaded_textures = NULL;
			if (texture_mappings.find(texture_type) == texture_mappings.end())
			{
				loaded_textures = new std::vector<Texture*>();
				texture_mappings[texture_type] = loaded_textures;
			}
			else
			{
				loaded_textures = texture_mappings[texture_type];
			}
			loaded_textures->push_back(texture);
		}
	}
}
/*
void AssimpLoader::clear()
{
	m_vertices_.clear();
	m_tex_coords_.clear();
	m_indices_.clear();
	m_normals_.clear();

	animations_.clear();
	bones_.clear();
	texture_mappings_.clear();
}
*/
