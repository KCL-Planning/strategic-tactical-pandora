#include <fstream>
#include <iostream>

#include "dpengine/models/AnimatedModel.h"
#include "dpengine/entities/Entity.h"
#include "dpengine/models/Bone.h"
#include "dpengine/models/Animation.h"
#include "dpengine/models/AnimationNode.h"
#include "dpengine/models/BoneNode.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace DreadedPE
{

AnimatedModel::AnimatedModel()
	: Shape(), current_animation_(NULL), next_animation_(NULL), wait_for_current_animation_to_finish_(false), wait_for_next_animation_to_finish_(false), animation_time_(0.0f)
{
	
}

AnimatedModel::AnimatedModel(const std::vector<glm::vec3>& m_vertices,
                             const std::vector<glm::vec2>& m_tex_coords,
                             const std::vector<GLuint>& m_indices,
                             const std::vector<glm::vec3>& m_normals, 
                             BoneNode& root_node,
                             const std::vector<Animation*>& animations,
                             const std::vector<Bone*>& bones)
	: Shape(m_vertices, m_tex_coords, m_indices, m_normals), root_node_(&root_node), animations_(animations), current_animation_(NULL), animation_time_(0.0f)
{
	bones_ = bones;
	finalise();
}

void AnimatedModel::finalise()
{
	// Initialise the bone buffers.
	m_bone_ids_.resize(m_vertices_.size(), glm::ivec4(0, 0, 0, 0));
	m_bone_weights_.resize(m_vertices_.size(), glm::vec4(0.0f, 0.0f, 0.0f, 0.0f));
	m_bone_ids2_.resize(m_vertices_.size(), glm::ivec4(0, 0, 0, 0));
	m_bone_weights2_.resize(m_vertices_.size(), glm::vec4(0.0f, 0.0f, 0.0f, 0.0f));
	
	unsigned int* vertex_bones = new unsigned int[m_vertices_.size()];
	memset(vertex_bones, 0, sizeof(unsigned int) * m_vertices_.size());
	unsigned int errors = 0;
	unsigned int max_error = 0;

	// For each bone, we check which verteces are affected and update the buffers accordingly.
	for (unsigned int i = 0; i < bones_.size(); ++i)
	{
		Bone* bone = bones_[i];
		const std::vector<std::pair<unsigned int, float> >& weights = bone->getVertexWeights();

		for (std::vector<std::pair<unsigned int, float> >::const_iterator ci = weights.begin(); ci != weights.end(); ++ci)
		{
			if (m_bone_weights2_[(*ci).first][3] != 0.0f)
			{
				++errors;
			}
			vertex_bones[(*ci).first] = vertex_bones[(*ci).first] + 1;

#ifdef _WIN32
			max_error = std::max(max_error, vertex_bones[(*ci).first]);
#endif
#ifdef __linux__
			max_error = std::max(max_error, vertex_bones[(*ci).first]);
#endif

			bool found_entry = false;
			for (unsigned int j = 0; j < 4; ++j)
			{
				if (m_bone_weights_[(*ci).first][j] == 0.0f)
				{
					m_bone_ids_[(*ci).first][j] = i;
					m_bone_weights_[(*ci).first][j] = (*ci).second;
					found_entry = true;
					break;
				}
			}

			if (!found_entry)
			{
				for (unsigned int j = 0; j < 4; ++j)
				{
					if (m_bone_weights2_[(*ci).first][j] == 0.0f)
					{
						m_bone_ids2_[(*ci).first][j] = i;
						m_bone_weights2_[(*ci).first][j] = (*ci).second;
						break;
					}
				}
			}
		}
	}

	if (errors != 0 || bones_.size() > 100)
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "Too many bones! This happened " << errors << " times! Max=" << max_error << ". Bones = " << bones_.size() << std::endl;
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	
	// Generate buffers.
	glGenBuffers(1, &m_vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	glGenBuffers(1, &m_index_buffer_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_index_buffer_); //Bind the vertex buffer
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// Generate the buffer for the texture coordinates.
	glGenBuffers(1, &m_tex_coord_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_tex_coords_.size(), &m_tex_coords_[0], GL_STATIC_DRAW);

	// Generate the buffers to store the normals.
	glGenBuffers(1, &m_normal_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_normals_.size(), &m_normals_[0], GL_STATIC_DRAW);

	// Generate the buffers to store the bone ids.
	glGenBuffers(1, &m_bone_ids_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_bone_ids_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLint) * 4 * m_bone_ids_.size(), &m_bone_ids_[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &m_bone_ids2_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_bone_ids2_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLint) * 4 * m_bone_ids2_.size(), &m_bone_ids2_[0], GL_STATIC_DRAW);

	// Generate the buffers to store the weights of the bones.
	glGenBuffers(1, &m_bone_weight_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_bone_weight_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * m_bone_weights_.size(), &m_bone_weights_[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &m_bone_weight2_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_bone_weight2_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * m_bone_weights2_.size(), &m_bone_weights2_[0], GL_STATIC_DRAW);
}

void AnimatedModel::prepare(float dt)
{
	if (current_animation_ == NULL)
	{
		return;
	}
	animation_time_ += dt;

	if (!wait_for_current_animation_to_finish_ || animation_time_ > current_animation_->getDuration() / current_animation_->getTicksPerSecond())
	{
		if (next_animation_ != current_animation_)
		{
			animation_time_ = 0.0f;
			current_animation_ = next_animation_;
			wait_for_current_animation_to_finish_ = wait_for_next_animation_to_finish_;
			current_animation_->setAsActiveAnimation();
		}
	}

	// Update the vertice positions and normals.
	float ticks_per_second = current_animation_->getTicksPerSecond();
	if (ticks_per_second == 0.0f)
	{
		ticks_per_second = 0.25f;
	}

	float ticks = ticks_per_second * animation_time_;
	float animation_duration = fmod(ticks, current_animation_->getDuration());

	animation_duration_ = animation_duration;

	// Update the nodes.
	root_node_->prepare(animation_duration);
}

void AnimatedModel::setAnimation(Animation& animation, bool wait_for_animation_to_finish)
{
	if (current_animation_ == &animation)
		return;

	//previous_animation_ = current_animation_;
	wait_for_next_animation_to_finish_ = wait_for_animation_to_finish;

	if (current_animation_ == NULL)
	{
		current_animation_ = &animation;
		// Update all the bones.
		animation.setAsActiveAnimation();

		wait_for_current_animation_to_finish_ = wait_for_animation_to_finish;
	}
	next_animation_ = &animation;
}

};
