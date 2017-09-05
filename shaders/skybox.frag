#version 420

uniform samplerCube texture0;
in vec3 texCoord;
out vec4 outColor;

void main(void) {
	outColor = texture(texture0, texCoord) * vec4(0.2, 0.2, 0.2, 1.0);
}
