#version 420

uniform sampler2D texture0;

varying vec2 texCoord0;

void main(void) {
	vec4 outColor = texture2D(texture0, texCoord0.st);
	
	if (outColor.a == 0.0) {
		discard;
	}
	gl_FragColor = outColor;
	
	//gl_FragColor = vec4(1, 0, 0, 1);
}
