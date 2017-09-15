# knowledge_msgs/Notification.msg

uint8 ADDED = 0
uint8 REMOVED = 1

uint8 function                  # notification type

# item spec
string type_name                # (required) specifies the knowledge class
string instance_name            # (optional) specifies the knowledge item (single instance)

# data property
string data_property_name       # (optional) item data property name
string data_property_value      # (optional) item data property value

# object property
string obj_property_name        # (optional) object property name (it describes the name of the relation)
string obj_property_type        # (optional) object property class (it must be a valid type_name)
string obj_property_value       # (optional) object property item (it must be a valid instance_name)
