#version 450

subroutine vec3 colorType();
subroutine uniform colorType colorUniform;

in vec3 color;

uniform vec3 unifColor;

layout (location = 0) out vec4 fColor;

subroutine(colorType)
vec3 normalColor()
{
	return color;
}

subroutine(colorType)
vec3 uniformColor()
{
	return unifColor;
}

void main() {
	fColor = vec4(colorUniform(), 1.0f);
}