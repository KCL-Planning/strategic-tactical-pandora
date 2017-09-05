#version 420 core

uniform mat4 modelviewprojection_matrix;

layout (location = 0) in vec4 a_Vertex;
layout (location = 1) in vec2 a_TexCoord0;

const int MAX_BONES = 100;
uniform mat4 bone_matrix[MAX_BONES];

in ivec4 a_bones_id;
in vec4 a_bone_weights;
in ivec4 a_bones_id_2;
in vec4 a_bone_weights_2;

out vec2 texCoord0;

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
	gl_Position = modelviewprojection_matrix * bone_transform * a_Vertex;
}
