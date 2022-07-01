#version 450

#extension GL_ARB_compute_variable_group_size: enable

layout (local_size_variable) in;
								
#include <Assets/Shaders/Compute/Templates/constraints.glsl>
#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

// Model 1
layout (std430, binding = 0) buffer VertexBuffer1	{ VertexGPUData				vertexMesh1[]; };
layout (std430, binding = 1) buffer FaceBuffer1		{ FaceGPUData				faceMesh1[]; };
layout (std430, binding = 2) buffer MeshDataBuffer1	{ MeshGPUData				meshMesh1[]; };

layout (std430, binding = 3) buffer ClusterBuffer	{ BVHCluster				clusterData[]; };
layout (std430, binding = 4) buffer VertexBuffer2	{ VertexGPUData				vertexMesh2[]; };
layout (std430, binding = 5) buffer FaceBuffer2		{ FaceGPUData				faceMesh2[]; };
layout (std430, binding = 6) buffer MeshDataBuffer2	{ MeshGPUData				meshMesh2[]; };

layout (std430, binding = 7) buffer CollisionBuffer	{ FaceGPUData				collidedFace[]; };
layout (std430, binding = 8) buffer CountCollision	{ uint						numCollisions; };

uniform uint		numClusters;					// Start traversal from last cluster (root)
uniform uint		numTriangles;

uniform mat4		mModelSource;
uniform mat4		mModelDest;

// Computes intersection between a ray and an axis-aligned bounding box. Slabs method
bool rayAABBIntersection(in RayGPUData ray, vec3 minPoint, vec3 maxPoint)
{
	vec3 tMin	= (minPoint - ray.origin) / ray.direction;
    vec3 tMax	= (maxPoint - ray.origin) / ray.direction;
    vec3 t1		= min(tMin, tMax);
    vec3 t2		= max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar	= min(min(t2.x, t2.y), t2.z);

    return tFar >= tNear && tNear < ray.destination.x;
}

// Computes intersection between a ray and a triangle
bool rayTriangleIntersection(uint index, float minDistance, in FaceGPUData face, in RayGPUData ray, in BVHCluster cluster)
{
	const vec3 v1 = vertexMesh2[face.vertices.x + meshMesh2[face.modelCompID].startIndex].position,
			   v2 = vertexMesh2[face.vertices.y + meshMesh2[face.modelCompID].startIndex].position,
			   v3 = vertexMesh2[face.vertices.z + meshMesh2[face.modelCompID].startIndex].position;

	vec3 edge1, edge2, h, s, q;
	float a, f, u, v, uv;

	edge1 = v2 - v1;
	edge2 = v3 - v1;

	h = cross(ray.direction, edge2);
	a = dot(edge1, h);

	if (abs(a) < EPSILON)				// Parallel ray case
	{
		return false;
	}

	f = 1.0f / a;
	s = ray.origin - v1;
	u = f * dot(s, h);

	if (u < 0.0f || u > 1.0f)
	{
		return false;
	}

	q = cross(s, edge1);
	v = f * dot(ray.direction, q);
	uv = u + v;

	if (v < 0.0f || uv > 1.0f)
	{
		return false;
	}

	float t	= f * dot(edge2, q);

	if (t >= -EPSILON && t < minDistance && t < ray.destination.x)
	{
		return true;
	}

	return false;							// There is a LINE intersection but no RAY intersection
}


void main()
{
	const uint index = gl_GlobalInvocationID.x;
	if (index >= numTriangles * 3)
	{
		return;
	}

	const uint triangleIndex	= uint(floor(index / 3)), offset = index % 3;
	const uint vertexStartIndex = meshMesh1[faceMesh1[triangleIndex].modelCompID].startIndex;
	FaceGPUData face			= faceMesh1[triangleIndex];
	
	// Build ray to be throw
	RayGPUData ray;
	ray.origin			= vec3(mModelDest * mModelSource * vec4(vertexMesh1[face.vertices[offset] + vertexStartIndex].position, 1.0f));
	ray.direction		= vec3(mModelDest * mModelSource * vec4(vertexMesh1[face.vertices[(offset + 1) % 3] + vertexStartIndex].position, 1.0f)) - ray.origin;
	ray.destination.x	= length(ray.direction);
	ray.direction		= normalize(ray.direction);

	// Initialize stack
	float	minDistance			= UINT_MAX;
	uint	collidedFaceIndex	= UINT_MAX;
	int		currentIndex		= 0;
	uint	toExplore[100];

	toExplore[currentIndex] = numClusters - 1;			// First node to explore: root

	while (currentIndex >= 0)
	{
		BVHCluster cluster = clusterData[toExplore[currentIndex]];

		if (rayAABBIntersection(ray, cluster.minPoint, cluster.maxPoint))
		{
			if (cluster.faceIndex != UINT_MAX)
			{
				if (rayTriangleIntersection(index, minDistance, faceMesh2[cluster.faceIndex], ray, cluster))
				{
					collidedFaceIndex = cluster.faceIndex;
				}
			}
			else
			{
				toExplore[currentIndex]		= cluster.prevIndex1;
				toExplore[++currentIndex]	= cluster.prevIndex2;
				++currentIndex;												// Prevent --currentIndex instead of branching
			}
		}

		--currentIndex;
	}

	if (collidedFaceIndex != UINT_MAX)
	{
		uint faceIndex = atomicAdd(numCollisions, 1);
		collidedFace[faceIndex] = face;
	}
}