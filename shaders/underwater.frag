#version 420

uniform float time;
uniform sampler2D fbo_texture;

in vec2 texCoord0;
out vec4 outColor;

const float blurSize = 1.0/512.0;

void main()
{
	outColor = texture2D(fbo_texture, vec2(texCoord0.s + sin(texCoord0.t + time) / 100, texCoord0.t)) * 0.5;

	// Blur it :).
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 3 * blurSize, texCoord0.t)) * 0.01;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 2 * blurSize, texCoord0.t)) * 0.02;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 1 * blurSize, texCoord0.t)) * 0.03;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 1 * blurSize, texCoord0.t)) * 0.03;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 2 * blurSize, texCoord0.t)) * 0.02;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 3 * blurSize, texCoord0.t)) * 0.11;

	outColor += texture2D(fbo_texture, vec2(texCoord0.s, texCoord0.t + 3 * blurSize)) * 0.01;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s, texCoord0.t + 2 * blurSize)) * 0.02;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s, texCoord0.t + 1 * blurSize)) * 0.03;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s, texCoord0.t - 1 * blurSize)) * 0.03;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s, texCoord0.t - 2 * blurSize)) * 0.02;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s, texCoord0.t - 3 * blurSize)) * 0.01;
	
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 1 * blurSize, texCoord0.t + 1 * blurSize)) * 0.015;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 1 * blurSize, texCoord0.t + 2 * blurSize)) * 0.025;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 2 * blurSize, texCoord0.t + 1 * blurSize)) * 0.025;

	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 1 * blurSize, texCoord0.t - 1 * blurSize)) * 0.015;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 1 * blurSize, texCoord0.t - 2 * blurSize)) * 0.025;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s + 2 * blurSize, texCoord0.t - 1 * blurSize)) * 0.025;

	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 1 * blurSize, texCoord0.t + 1 * blurSize)) * 0.015;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 1 * blurSize, texCoord0.t + 2 * blurSize)) * 0.025;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 2 * blurSize, texCoord0.t + 1 * blurSize)) * 0.025;

	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 1 * blurSize, texCoord0.t - 1 * blurSize)) * 0.015;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 1 * blurSize, texCoord0.t - 2 * blurSize)) * 0.025;
	outColor += texture2D(fbo_texture, vec2(texCoord0.s - 2 * blurSize, texCoord0.t - 1 * blurSize)) * 0.025;

	outColor *= vec4(0.7, 0.7, 1.0, 1.0);
}
