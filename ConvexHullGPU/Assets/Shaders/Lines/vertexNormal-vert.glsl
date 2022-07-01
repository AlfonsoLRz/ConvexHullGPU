#version 450

layout (location = 0) in vec4 vPosition;
layout (location = 1) in vec3 vNormal;

uniform mat4	mModelViewProj;

out VS_OUT {
    vec4 normal;
	vec3 color;
} vs_out;

void main() {
	gl_Position		= mModelViewProj * vPosition;
	vs_out.normal	= mModelViewProj * vec4(normalize(vNormal), 0.0f);
	vs_out.color	= normalize((vNormal + 1.0f) / 2.0f);
}