#include "FBXLoader.h"

#include <iostream>

int FBXLoader::numTabs = 0;

std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > FBXLoader::LoadModel(SceneManager& scene_manager, const std::string& filename)
{
	// Initialize the SDK manager. This object handles all our memory management.
	FbxManager* lSdkManager = FbxManager::Create();
	
	numTabs = 0;
	FbxIOSettings *ios = FbxIOSettings::Create(lSdkManager, IOSROOT);
	lSdkManager->SetIOSettings(ios);

	// Create an importer using the SDK manager.
	FbxImporter* lImporter = FbxImporter::Create(lSdkManager,"");
	
	// Use the first argument as the filename for the importer.
	if(!lImporter->Initialize(filename.c_str(), -1, lSdkManager->GetIOSettings()))
	{ 
		std::cout << "Call to FbxImporter::Initialize() failed." << std::endl; 
		std::cout << "Error returned: " << lImporter->GetStatus().GetErrorString() << std::endl << std::endl;
		exit(-1);
	}
	
	// Create a new scene so that it can be populated by the imported file.
	FbxScene* lScene = FbxScene::Create(lSdkManager,"myScene");

	// Import the contents of the file into the scene.
	lImporter->Import(lScene);

	// The file is imported, so get rid of the importer.
	lImporter->Destroy();
	
	// Print the nodes of the scene and their attributes recursively.
	// Note that we are not printing the root node because it should
	// not contain any attributes.
	FbxNode* lRootNode = lScene->GetRootNode();
	if(lRootNode) {
		for(int i = 0; i < lRootNode->GetChildCount(); i++)
			PrintNode(lRootNode->GetChild(i));
	}
	// Destroy the SDK manager and all the other objects it was handling.
	lSdkManager->Destroy();
	
	return std::make_pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* >(NULL, NULL);
}

/**
 * Print a node, its attributes, and all its children recursively.
 */
void FBXLoader::PrintNode(FbxNode* pNode)
{
    PrintTabs();
    const char* nodeName = pNode->GetName();
    FbxDouble3 translation = pNode->LclTranslation.Get(); 
    FbxDouble3 rotation = pNode->LclRotation.Get(); 
    FbxDouble3 scaling = pNode->LclScaling.Get();

    // Print the contents of the node.
    std::cout << "<node name='" << nodeName << " translation=(" << translation[0] << ", " << translation[1] << ", " << translation[2] << ")' rotation='(" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ")' scaling='(" << scaling[0] << ", " << scaling[1] << ", " << scaling[2] << ")'>" << std::endl;
    numTabs++;

    // Print the node's attributes.
    for(int i = 0; i < pNode->GetNodeAttributeCount(); i++)
        PrintAttribute(pNode->GetNodeAttributeByIndex(i));

    // Recursively print the children.
    for(int j = 0; j < pNode->GetChildCount(); j++)
        PrintNode(pNode->GetChild(j));

    numTabs--;
    PrintTabs();
    std::cout << "</node>" << std::endl;
}

/**
 * Print the required number of tabs.
 */
void FBXLoader::PrintTabs() {
	for(int i = 0; i < numTabs; i++)
		printf("\t");
}

/**
 * Return a string-based representation based on the attribute type.
 */
FbxString FBXLoader::GetAttributeTypeName(FbxNodeAttribute::EType type) { 
	switch(type) { 
		case FbxNodeAttribute::eUnknown: return "unidentified"; 
		case FbxNodeAttribute::eNull: return "null"; 
		case FbxNodeAttribute::eMarker: return "marker"; 
		case FbxNodeAttribute::eSkeleton: return "skeleton"; 
		case FbxNodeAttribute::eMesh: return "mesh"; 
		case FbxNodeAttribute::eNurbs: return "nurbs"; 
		case FbxNodeAttribute::ePatch: return "patch"; 
		case FbxNodeAttribute::eCamera: return "camera"; 
		case FbxNodeAttribute::eCameraStereo: return "stereo"; 
		case FbxNodeAttribute::eCameraSwitcher: return "camera switcher"; 
		case FbxNodeAttribute::eLight: return "light"; 
		case FbxNodeAttribute::eOpticalReference: return "optical reference"; 
		case FbxNodeAttribute::eOpticalMarker: return "marker"; 
		case FbxNodeAttribute::eNurbsCurve: return "nurbs curve"; 
		case FbxNodeAttribute::eTrimNurbsSurface: return "trim nurbs surface"; 
		case FbxNodeAttribute::eBoundary: return "boundary"; 
		case FbxNodeAttribute::eNurbsSurface: return "nurbs surface"; 
		case FbxNodeAttribute::eShape: return "shape"; 
		case FbxNodeAttribute::eLODGroup: return "lodgroup"; 
		case FbxNodeAttribute::eSubDiv: return "subdiv"; 
		default: return "unknown"; 
	} 
}

/**
 * Print an attribute.
 */
void FBXLoader::PrintAttribute(FbxNodeAttribute* pAttribute) {
	if(!pAttribute) return;

	FbxString typeName = GetAttributeTypeName(pAttribute->GetAttributeType());
	FbxString attrName = pAttribute->GetName();
	PrintTabs();
	// Note: to retrieve the character array of a FbxString, use its Buffer() method.
	printf("<attribute type='%s' name='%s'/>\n", typeName.Buffer(), attrName.Buffer());
}
