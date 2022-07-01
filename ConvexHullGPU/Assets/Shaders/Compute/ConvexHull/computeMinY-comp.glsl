#version 450

#extension GL_ARB_compute_variable_group_size : enable
#extension GL_NV_gpu_shader5 : enable
#extension GL_ARB_gpu_shader_int64 : require
#extension GL_NV_shader_atomic_int64 : require

layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer VertexBuffer	{ VertexGPUData vertex[]; };
layout (std430, binding = 1) buffer CollapseBuffer	{ uint64_t minY; };

uniform vec3 aabbMin;
uniform uint arraySize;

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= arraySize) return;

	uint64_t y = floatBitsToUint(vertex[index].position.y + aabbMin.y);
	atomicMin(minY, (y << 32) | index);
}