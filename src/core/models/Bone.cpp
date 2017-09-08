#include "dpengine/models/Bone.h"

namespace DreadedPE
{

Bone::Bone(const std::string& bone_name, const glm::mat4& offset_matrix)
	: bone_name_(bone_name), offset_matrix_(offset_matrix)
{

}

void Bone::addVertexWeight(unsigned int vertex_index, float weight)
{
	vertex_to_weight_.push_back(std::make_pair(vertex_index, weight));
}

void Bone::setFinalTransformation(const glm::mat4& final_transformation)
{
	final_transformation_ = final_transformation;
}

};
