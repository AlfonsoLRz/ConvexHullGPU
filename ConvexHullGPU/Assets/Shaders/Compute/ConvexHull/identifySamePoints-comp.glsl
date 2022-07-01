#version 450

#extension GL_ARB_compute_variable_group_size : enable
#extension GL_NV_gpu_shader5 : enable
layout (local_size_variable) in;

#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer VertexBuffer	{ VertexGPUData vertex[]; };
layout (std430, binding = 1) buffer CollapseBuffer	{ uint8_t collapse[]; };

uniform uint arraySize;
uniform uint radius;

void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= arraySize) return;

	for (int i = 1; i < radius; ++i)
	{
		if (distance(vertex[index].position, vertex[index + i].position) < .001f)
		{
			collapse[index + i] = uint8_t(1);
		}
	}
}