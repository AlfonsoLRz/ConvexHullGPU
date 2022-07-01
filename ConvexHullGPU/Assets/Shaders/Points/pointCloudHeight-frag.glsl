#version 450

in vec4 worldPosition;

uniform sampler2D	texKadSampler;
uniform vec2		heightBoundaries;
uniform float		maxSceneHeight;
uniform float		minSceneHeight;

layout (location = 0) out vec4 fColor;

void main() {
	// Rounded points
	vec2 centerPointv = gl_PointCoord - 0.5f;
	if (dot(centerPointv, centerPointv) > 0.25f)		// Vector * vector = square module => we avoid square root
	{
		discard;										// Discarded because distance to center is bigger than 0.5
	}

	float heightInterp = (worldPosition.y - minSceneHeight) / (maxSceneHeight - minSceneHeight) * (heightBoundaries.y - heightBoundaries.x) + heightBoundaries.x;
	fColor = texture(texKadSampler, vec2(.5f, heightInterp));
}