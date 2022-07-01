// Computes intersection between a ray and an axis-aligned bounding box. Slabs method
bool rayAABBIntersection(in RayGPUData ray, vec3 minPoint, vec3 maxPoint)
{
	vec3 tMin		= (minPoint - ray.origin) / ray.direction;
	vec3 tMax		= (maxPoint - ray.origin) / ray.direction;
	vec3 t1			= min(tMin, tMax);
	vec3 t2			= max(tMin, tMax);
	float tNear		= max(max(t1.x, t1.y), t1.z);
	float tFar		= min(min(t2.x, t2.y), t2.z);

	return tFar >= tNear;
}

// Computes intersection between a ray and a triangle
bool rayTriangleIntersection(FaceGPUData face, RayGPUData ray, BVHCluster cluster, inout float minDistance)
{
	const VertexGPUData v1 = vertexData[face.vertices.x + meshData[face.modelCompID].startIndex],
						v2 = vertexData[face.vertices.y + meshData[face.modelCompID].startIndex],
						v3 = vertexData[face.vertices.z + meshData[face.modelCompID].startIndex];

	vec3 edge1, edge2, h, s, q;
	float a, f, u, v, uv;

	edge1 = v2.position - v1.position;
	edge2 = v3.position - v1.position;

	h = cross(ray.direction, edge2);
	a = dot(edge1, h);

	if (abs(a) < EPSILON)				// Parallel ray case
	{
		return false;
	}

	f = 1.0f / a;
	s = ray.origin - v1.position;
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

	float t = f * dot(edge2, q);
	vec3 intersectionPoint = ray.origin + ray.direction * t;
	float distanceToOrigin = distance(ray.origin, intersectionPoint);

	if (t >= -EPSILON && t < minDistance)
	{
		minDistance = distanceToOrigin;

		return true;
	}

	return false;							// There is a LINE intersection but no RAY intersection
}