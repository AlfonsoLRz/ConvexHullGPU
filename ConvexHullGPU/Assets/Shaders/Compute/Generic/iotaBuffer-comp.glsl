#version 450

#extension GL_ARB_compute_variable_group_size: enable

layout (local_size_variable) in;

layout(std430, binding = 0) buffer InputBuffer { uint bufferData[]; };

uniform uint numIndices;
uniform uint value;

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= numIndices) return;

	bufferData[index] = value;
}