#version 420

uniform float time;
uniform sampler2D fbo_texture;
uniform sampler2D screen_texture;

in vec2 texCoord0;
out vec4 outColor;

void main()
{
	vec4 colour1 = texture2D(fbo_texture, texCoord0.st);
	vec4 colour2 = texture2D(screen_texture, texCoord0.st);

	outColor = colour1 * (1 - colour2.a) + colour2;
	//outColor = texture2D(fbo_texture, texCoord0.st) * 0.7 + texture2D(screen_texture, texCoord0.st) * 0.3;
}
