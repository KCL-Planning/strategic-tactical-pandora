# knowledge_msgs/Filter.msg

uint8 F_CLEAR = 0
uint8 F_INSERT = 1
uint8 F_REMOVE = 2
uint8 F_INSERT_DATA_ATTR = 3
uint8 F_REMOVE_DATA_ATTR = 4
uint8 F_INSERT_OBJ_ATTR = 5
uint8 F_REMOVE_OBJ_ATTR = 6

uint8 function                  # filter function (add, clear, remove)

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
