#version 420

uniform float exposure;
uniform float decay;
uniform float density;
uniform float weight;
uniform sampler2D fbo_texture;
uniform sampler2D screen_texture;
uniform vec4 light_position;

const int NUM_SAMPLES = 100;

in vec2 texCoord0;

void main()
{
	if (light_position.x == -1)
	{
		gl_FragColor = texture2D(screen_texture, texCoord0.st);
	}
	else
	{
		vec2 deltaTextCoord = vec2( texCoord0.st - light_position.xy );
		vec2 textCoo = texCoord0.st;
		deltaTextCoord *= 1.0 / float(NUM_SAMPLES) * density;
		float illuminationDecay = 1.0;

		for(int i=0; i < NUM_SAMPLES ; i++)
		{
			textCoo -= deltaTextCoord;
			vec4 sample = texture2D(fbo_texture, textCoo );
			sample *= illuminationDecay * weight;
			gl_FragColor += sample;
			illuminationDecay *= decay;
		}
		gl_FragColor *= exposure;
		gl_FragColor /= NUM_SAMPLES;

		gl_FragColor *= 0.4;
		gl_FragColor += texture2D(screen_texture, texCoord0.st);
	}

	if (length(texCoord0.st - light_position.xy) < .001)
	{
		gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
	}
}

