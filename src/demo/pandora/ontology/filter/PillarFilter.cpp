#include <sstream>

#include "PillarFilter.h"
#include "../../structures/Pillar.h"

PillarFilter::PillarFilter(const knowledge_msgs::Filter& filter_msg, Pillar& pillar)
	: Filter(filter_msg), pillar_(&pillar)
{
	if (pillar.notifactionHasBeenSent())
	{
		std::cout << "The new pillar filter for " << pillar.getName() << " will NEVER TRIGGER!" << std::endl;
	}
	else
	{
		std::cout << "The new pillar filter for " << pillar.getName() << " might trigger!" << std::endl;
	}
}

bool PillarFilter::checkFilter()
{
	if (!pillar_->notifactionHasBeenSent() && pillar_->hasBeenObserved())
	{
		std::cout << pillar_->getName() << " is observed!" << std::endl;
		pillar_->setNotificationSent(true);
		return false;
	}
	return true;
}

knowledge_msgs::Notification PillarFilter::prepareNotification()
{
	knowledge_msgs::Notification notification;
	
	// TODO: This isn't correct I think.
	if (filter_msg_.function == knowledge_msgs::Filter::F_INSERT || filter_msg_.function == knowledge_msgs::Filter::F_INSERT_DATA_ATTR || filter_msg_.function == knowledge_msgs::Filter::F_INSERT_OBJ_ATTR)
	{
		notification.function = knowledge_msgs::Notification::ADDED;
	}
	else
	{
		notification.function = knowledge_msgs::Notification::REMOVED;
	}

	notification.type_name = filter_msg_.type_name;
	notification.instance_name = pillar_->getName()	;
	
	notification.data_property_name = filter_msg_.data_property_name;
	notification.data_property_value = filter_msg_.data_property_value;
	notification.obj_property_name = filter_msg_.obj_property_name;
	notification.obj_property_type = filter_msg_.obj_property_type;
	notification.obj_property_value = filter_msg_.obj_property_value;
	return notification;
}
