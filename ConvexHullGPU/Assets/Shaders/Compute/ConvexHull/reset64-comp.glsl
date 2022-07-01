#version 450

#extension GL_ARB_compute_variable_group_size : enable
#extension GL_NV_gpu_shader5 : enable
layout (local_size_variable) in;

layout (std430, binding = 0) buffer UintBuffer	{ uint64_t valueBuffer[]; };

uniform uint arraySize;
uniform uint value;

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= arraySize) return;

	valueBuffer[index] = uint64_t(value) << 32;
}