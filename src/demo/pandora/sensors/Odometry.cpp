#include <sstream>
#include "Odometry.h"
#include <tf/transform_broadcaster.h>

#include "../../../core/entities/Entity.h"

Odometry::Odometry(Entity& entity)
	: entity_(&entity)
{

}

void Odometry::update(float dt)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(-entity_->getGlobalLocation().x, entity_->getGlobalLocation().z, entity_->getGlobalLocation().y) );
//	tf::Quaternion q(entity_->getGlobalRotation().w, entity_->getGlobalRotation().x, entity_->getGlobalRotation().y, entity_->getGlobalRotation().z);

	glm::fquat rot = entity_->getGlobalRotation();
	tf::Quaternion q(-rot.x, rot.z, rot.y, rot.w);
	//q.setEulerZYX(glm::roll(entity_->getGlobalRotation()), glm::pitch(entity_->getGlobalRotation()), glm::yaw(entity_->getGlobalRotation()));
	//q.setEulerZYX(glm::yaw(entity_->getGlobalRotation()) * M_PI / 180.0f, glm::pitch(entity_->getGlobalRotation()) * M_PI / 180.0f, glm::roll(entity_->getGlobalRotation()) * M_PI / 180.0f);
	//q.setEulerZYX(glm::yaw(entity_->getGlobalRotation()), glm::pitch(entity_->getGlobalRotation()), glm::roll(entity_->getGlobalRotation()));
	transform.setRotation(q);
	//transform.setOrigin(tf::Vector3(-transform.getOrigin().getX(), transform.getOrigin().getZ(), transform.getOrigin().getY()));
	std::stringstream ss;
	ss << entity_->getName() << "/location";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", ss.str()));
}
