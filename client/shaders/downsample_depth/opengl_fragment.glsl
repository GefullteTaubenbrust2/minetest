#define depthmap texture0

uniform sampler2D depthmap;
uniform vec2 texelSize0;

CENTROID_ VARYING_ mediump vec2 varTexCoord;

void main(void)
{
	float depth = 0.0;
	for (float x = 0.0; x < 1.0; x += 1.0 / (float(VOLUMETRICS_UNDERSAMPLING))) {
		for (float y = 0.0; y < 1.0; y += 1.0 / (float(VOLUMETRICS_UNDERSAMPLING))) {
			vec2 coord = varTexCoord + texelSize0 * vec2(x, y);
			float s = texture2D(depthmap, coord).r * 0.1;
			if (s > depth) depth = s;
		}
	}

	gl_FragColor = vec4(depth, 0.0, 0.0, 1.0);
}
