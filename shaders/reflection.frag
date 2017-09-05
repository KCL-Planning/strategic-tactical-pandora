#version 420

uniform samplerCube texture0;

uniform vec3 camPosition;

in vec3 modelPos;
in vec3 modelNormal;
in vec3 nor;

out vec4 outColor;

void main(void) {

	vec3 v = modelPos - camPosition;
	vec3 reflection = reflect(v, normalize(modelNormal));
	outColor = texture(texture0, reflection);
}
