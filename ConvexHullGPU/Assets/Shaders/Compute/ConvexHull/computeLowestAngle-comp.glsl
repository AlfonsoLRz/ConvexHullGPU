#version 450

#extension GL_ARB_compute_variable_group_size : enable
#extension GL_NV_gpu_shader5 : enable
#extension GL_ARB_gpu_shader_int64 : require
#extension GL_NV_shader_atomic_int64 : require

layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer VertexBuffer	{ VertexGPUData vertex[]; };
layout (std430, binding = 1) buffer CollapseBuffer	{ uint64_t minY; };

uniform uint arraySize, startingPoint;

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= arraySize || index == startingPoint) return;

	vec3 point = vertex[index].position - vertex[startingPoint].position;
	float alpha = atan(point.y, point.x);
	uint64_t alphaUint = floatBitsToUint(alpha);
	atomicMin(minY, (alphaUint << 32) | index);
}