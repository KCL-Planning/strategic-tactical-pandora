# knowledge_msgs/KnowledgeObject.msg
#
#   this message type describe the return type for ontology queries

string type_name                                # specifies the knowledge class
string instance_name                            # specifies the knowledge item (single instance)

knowledge_msgs/DataProperty[] data_properties   # (optional) data attributes and object properties values
knowledge_msgs/ObjectProperty[] obj_properties  # (optional) data attributes and object properties values
