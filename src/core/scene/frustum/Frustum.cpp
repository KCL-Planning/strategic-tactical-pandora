#include "dpengine/scene/frustum/Frustum.h"

namespace DreadedPE
{

Frustum::Frustum(const glm::mat4& model_view_projection_matrix)
{
	setFrustum(model_view_projection_matrix);
}

Frustum::Frustum(const std::vector<glm::vec4>& planes)
	: planes_(planes)
{
	for (unsigned int i = 0; i < planes_.size(); i++)
	{
		finalise(i);
	}
}

void Frustum::setFrustum(const glm::mat4& mvp)
{
	planes_.clear();
	planes_.push_back(glm::vec4(mvp[0][3] + mvp[0][2], mvp[1][3] + mvp[1][2], mvp[2][3] + mvp[2][2], mvp[3][3] + mvp[3][2]));
	planes_.push_back(glm::vec4(mvp[0][3] - mvp[0][2], mvp[1][3] - mvp[1][2], mvp[2][3] - mvp[2][2], mvp[3][3] - mvp[3][2]));
	planes_.push_back(glm::vec4(mvp[0][3] + mvp[0][0], mvp[1][3] + mvp[1][0], mvp[2][3] + mvp[2][0], mvp[3][3] + mvp[3][0]));
	planes_.push_back(glm::vec4(mvp[0][3] - mvp[0][0], mvp[1][3] - mvp[1][0], mvp[2][3] - mvp[2][0], mvp[3][3] - mvp[3][0]));
	planes_.push_back(glm::vec4(mvp[0][3] + mvp[0][1], mvp[1][3] + mvp[1][1], mvp[2][3] + mvp[2][1], mvp[3][3] + mvp[3][1]));
	planes_.push_back(glm::vec4(mvp[0][3] - mvp[0][1], mvp[1][3] - mvp[1][1], mvp[2][3] - mvp[2][1], mvp[3][3] - mvp[3][1]));

	for (unsigned int i = 0; i < 6; ++i)
	{
		finalise(i);
	}
}

void Frustum::finalise(unsigned int i)
{
	float d = sqrt(planes_[i].x * planes_[i].x + planes_[i].y * planes_[i].y + planes_[i].z * planes_[i].z);
	planes_[i] = glm::vec4(planes_[i].x / d, planes_[i].y / d, planes_[i].z / d, planes_[i].w / d);
}

};
