#ifndef CORE_MODELS_BONE_H
#define CORE_MODELS_BONE_H

#include <vector>
#include <string>
#include <glm/glm.hpp>

class Bone
{
public:
	Bone(const std::string& bone_name, const glm::mat4& offset_matrix);
	void addVertexWeight(unsigned int vertex_index, float weight);

	const std::vector<std::pair<unsigned int, float> >& getVertexWeights() const { return vertex_to_weight_; }

	void setFinalTransformation(const glm::mat4& final_transformation);
	const glm::mat4& getFinalTransformation() const { return final_transformation_; }

	const glm::mat4& getOffsetMatrix() const { return offset_matrix_; }

	const std::string& getName() const { return bone_name_; }

private:
	std::string bone_name_;
	glm::mat4 final_transformation_;
	glm::mat4 offset_matrix_;
	std::vector<std::pair<unsigned int, float> > vertex_to_weight_;
};

#endif
