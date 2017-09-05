#include <sstream>

#include "ChainFilter.h"
#include "../ChainGoal.h"
#include "../../structures/Chain.h"
#include "../../level/MissionSite.h"
#include "../../level/Mission.h"

ChainFilter::ChainFilter(const knowledge_msgs::Filter& filter_msg, Chain& chain)
	: Filter(filter_msg), chain_(&chain)
{
	if (chain.notifactionHasBeenSent())
	{
		std::cout << "The new chain filter for chain " << chain.getName() << " will NEVER TRIGGER!" << std::endl;
	}
	else
	{
		std::cout << "The new chain filter for chain " << chain.getName() << " might trigger!" << std::endl;
	}
}

bool ChainFilter::checkFilter()
{
	if (!chain_->notifactionHasBeenSent() && chain_->hasBeenObserved())
	{
		std::cout << "CHAIN " << chain_->getName() << " is observed!" << std::endl;
		chain_->setNotificationSent(true);
		return false;
	}
	return true;
}

knowledge_msgs::Notification ChainFilter::prepareNotification()
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
	notification.instance_name = chain_->getName();
	
	notification.data_property_name = "Mission";
	if (chain_->getMissionSite().getMissions().size() != 1)
	{
		std::cerr << "The mission site for a chain should contain exactly one mission!" << std::endl;
		//exit(1);
	}
	
	notification.data_property_value = chain_->getMissionSite().getMissions()[0]->getId();
	notification.obj_property_name = filter_msg_.obj_property_name;
	notification.obj_property_type = filter_msg_.obj_property_type;
	notification.obj_property_value = filter_msg_.obj_property_value;
	return notification;
}
