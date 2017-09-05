/**
 * A specialised shape that uses instanced rendering.
 */
#ifndef DEMO_INSTANCE_RENDERING_INSTANCE_RENDERED_SHAPE_H
#define DEMO_INSTANCE_RENDERING_INSTANCE_RENDERED_SHAPE_H

#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>

#include "../../shapes/Shape.h"

class SceneNode;

/**
 * Wrapper class for the actual shape that is being rendered.
 */
class InstanceRenderedShape : public Shape
{
public:
	/**
	 * Create a shape that can be rendered multiple times, using a seperate model matrix for 
	 * each instance.
	 */
	InstanceRenderedShape(Shape& instance);

	/**
	 * Add an instance to be rendered.
	 * @param node The node where one of the shapes is at.
	 */
	//void addInstance(SceneNode& node);

	/**
	 * Overloaded render method, this will use the instanced render method instead of seperate rendering methods.
	 */
	void render();

	/**
	 * @return The buffer ID where the set of model matrixes are stored.
	 */
	GLuint getModelMatrixBufferId() const { return model_matrix_buffer_id_; }

	/**
	 * @return The list of model matrixes.
	 */
	const std::vector<glm::mat4>& getModelMatrixes() { return model_matrix_buffer_; }
	
	/**
	 * @return The scene nodes that are children of this class.
	 */
	//const std::vector<SceneNode*>& getNodes() const { return scene_nodes_; }
	
	/**
	 * @param model_matrixes Replace the model matrixes with the gieven list.
	 */
	void setModelMatrixes(const std::vector<glm::mat4>& matrixes);

	/**
	 * Update the buffers and send it to the GPU.
	 */
	void finaliseBuffers();
	
	/**
	 * Check if the shape is updated.
	 */
	//bool isUpdated() const { return needs_updating_; }
	
	/**
	 * Set the updating flag.
	 */
	//void setUpdated(bool needs_updating) { needs_updating_ = needs_updating; }
	
private:
	GLuint model_matrix_buffer_id_;
	std::vector<glm::mat4> model_matrix_buffer_;
	//bool needs_updating_;
};

#endif
