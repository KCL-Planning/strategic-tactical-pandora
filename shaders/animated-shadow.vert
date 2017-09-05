#version 420

#include shaders/light_header.inc
#include shaders/light_header_vert.inc

uniform mat4 projection_matrix;
uniform mat4 modelview_matrix;
uniform mat4 model_matrix;
uniform mat4 view_matrix;

in vec3 a_Vertex;
in vec2 a_TexCoord0;
in vec3 a_Normal;

const int MAX_BONES = 100;
uniform mat4 bone_matrix[MAX_BONES];

in ivec4 a_bones_id;
in vec4 a_bone_weights;
in ivec4 a_bones_id_2;
in vec4 a_bone_weights_2;

out vec2 texCoord0;
out float blendFactor;
out float height;
out vec4 worldPos;

void main(void) 
{
	mat4 bone_transform = mat4(1.0);
	
	if (a_bone_weights[0] != 0)
	{
		bone_transform = bone_matrix[a_bones_id[0]] * a_bone_weights[0];
		bone_transform += bone_matrix[a_bones_id[1]] * a_bone_weights[1];
		bone_transform += bone_matrix[a_bones_id[2]] * a_bone_weights[2];
		bone_transform += bone_matrix[a_bones_id[3]] * a_bone_weights[3];

		bone_transform += bone_matrix[a_bones_id_2[0]] * a_bone_weights_2[0];
		bone_transform += bone_matrix[a_bones_id_2[1]] * a_bone_weights_2[1];
		bone_transform += bone_matrix[a_bones_id_2[2]] * a_bone_weights_2[2];
		bone_transform += bone_matrix[a_bones_id_2[3]] * a_bone_weights_2[3];
	}
	
	texCoord0 = a_TexCoord0;
	vec4 pos = modelview_matrix * bone_transform * vec4(a_Vertex, 1.0);
	worldPos = model_matrix * bone_transform * vec4(a_Vertex, 1.0);
	
	#include shaders/light_body_vert.inc
	
	height = a_Vertex[1] / 10.0;

	blendFactor = 1 - length(pos) / 300;//exp2(-.1 * length(pos));
	gl_Position = projection_matrix * pos;
}
