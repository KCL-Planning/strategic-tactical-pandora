#include <iostream>

#include "InstanceRenderedShape.h"
#include "../../core/scene/SceneNode.h"

InstanceRenderedShape::InstanceRenderedShape(Shape& instance)
	: Shape(instance)//, needs_updating_(true)
{
	glGenBuffers(1, &model_matrix_buffer_id_);
}
/*
void InstanceRenderedShape::addInstance(SceneNode& scene_node)
{
	model_matrix_buffer_.push_back(scene_node.getCompleteTransformation());
	scene_nodes_.push_back(&scene_node);
}
*/
void InstanceRenderedShape::render()
{
	glDrawElementsInstanced(GL_TRIANGLES, getIndices().size(), GL_UNSIGNED_INT, 0, model_matrix_buffer_.size());
}

void InstanceRenderedShape::setModelMatrixes(const std::vector<glm::mat4>& matrixes)
{
	model_matrix_buffer_ = matrixes;
	finaliseBuffers();
	std::cout << "Update model matrix: " << matrixes.size() << std::endl;
}

void InstanceRenderedShape::finaliseBuffers()
{
	glBindBuffer(GL_ARRAY_BUFFER, model_matrix_buffer_id_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::mat4) * model_matrix_buffer_.size(), &model_matrix_buffer_[0], GL_DYNAMIC_DRAW);
}
