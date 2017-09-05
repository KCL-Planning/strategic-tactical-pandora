#include "Filter.h"

Filter::Filter(const knowledge_msgs::Filter& filter_msg)
	: filter_msg_(filter_msg), has_been_triggered_(false)
{
	
}
	
knowledge_msgs::Notification Filter::prepareNotification()
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
	notification.instance_name = filter_msg_.instance_name;
	
	notification.data_property_name = filter_msg_.data_property_name;
	notification.data_property_value = filter_msg_.data_property_value;
	notification.obj_property_name = filter_msg_.obj_property_name;
	notification.obj_property_type = filter_msg_.obj_property_type;
	notification.obj_property_value = filter_msg_.obj_property_value;
	return notification;
}

std::ostream& operator<<(std::ostream& o, const Filter& filter)
{
	o << "Filter: " << std::endl;
	o << "\tType name: " << filter.filter_msg_.type_name << std::endl;
	o << "\tInstance name: " << filter.filter_msg_.instance_name << std::endl;
	
	o << "\tData property: " << filter.filter_msg_.data_property_name <<  " = " << filter.filter_msg_.data_property_value << std::endl;
	o << "\tInstance: " << filter.filter_msg_.obj_property_name << "(" << filter.filter_msg_.obj_property_type << ") = " << filter.filter_msg_.obj_property_value << std::endl;
	return o;
}
