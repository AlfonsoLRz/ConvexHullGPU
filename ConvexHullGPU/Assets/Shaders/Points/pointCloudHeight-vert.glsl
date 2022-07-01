#version 450

layout (location = 0) in vec4 vPosition;

uniform mat4 mModelViewProj;
uniform float pointSize;	

out vec4 worldPosition;			// Needed to assign height

void main() {
	worldPosition = vPosition;
	gl_PointSize = pointSize;
	gl_Position = mModelViewProj * vPosition;
}