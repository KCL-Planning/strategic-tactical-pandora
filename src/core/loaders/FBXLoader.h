#ifndef CORE_LOADERS_FBX_LOADER_H
#define CORE_LOADERS_FBX_LOADER_H

#include <vector>
#include <string>
#include <map>

#include <fbxsdk.h>

#include <assimp_latest/material.h>

class AnimatedModel;
class SceneManager;
class Texture;

class FBXLoader
{
public:
	static std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > LoadModel(SceneManager& scene_manager, const std::string& filename);
private:
	static void PrintNode(FbxNode* pNode);
	static void PrintTabs();
	static FbxString GetAttributeTypeName(FbxNodeAttribute::EType type);
	static void PrintAttribute(FbxNodeAttribute* pAttribute);
	
	/* Tab character ("\t") counter */
	static int numTabs;
};

#endif
