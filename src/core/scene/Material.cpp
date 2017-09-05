#include "Material.h"

Material::Material(MaterialLightProperty& ambient, MaterialLightProperty& diffuse, MaterialLightProperty& specular, MaterialLightProperty& emissive, float transparency)
	: ambient_(ambient), diffuse_(diffuse), specular_(specular), emissive_(emissive), transparency_(transparency)
{

}
