#version 450

#extension GL_ARB_compute_variable_group_size : enable
#extension GL_NV_gpu_shader5 : enable
layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer VertexBuffer	{ VertexGPUData vertex[]; };
layout (std430, binding = 1) buffer CollapseBuffer	{ uint8_t collapse[]; };
layout (std430, binding = 2) buffer SupportBuffer	{ VertexGPUData newVertex[]; };
layout (std430, binding = 3) buffer CountBuffer		{ uint count; };

uniform uint arraySize;

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= arraySize) return;

	if (collapse[index] != uint8_t(1))
	{
		uint newIndex = atomicAdd(count, 1);
		newVertex[newIndex] = vertex[index];
	}
}