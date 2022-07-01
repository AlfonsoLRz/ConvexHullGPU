vec2 getTextCoord(vec3 position, in VertexGPUData vertex1, in VertexGPUData vertex2, in VertexGPUData vertex3)
{
	float u, v, w;

	const vec3 v0	= vertex2.position - vertex1.position, v1 = vertex3.position - vertex1.position, v2 = position - vertex1.position;
	float d00		= dot(v0, v0);
	float d01		= dot(v0, v1);
	float d11		= dot(v1, v1);
	float d20		= dot(v2, v0);
	float d21		= dot(v2, v1);
	float denom		= d00 * d11 - d01 * d01;
	v				= (d11 * d20 - d01 * d21) / denom;
	w				= (d00 * d21 - d01 * d20) / denom;
	u				= 1.0f - v - w;
	
	return u * vertex1.textCoord + v * vertex2.textCoord + w * vertex3.textCoord;
}

// Computes intersection between a ray and a triangle. This method considers
// integration within a compute shader which receives a faceData and vertextData
// array.
bool rayTriangleIntersection(const uint index, in FaceGPUData face, in RayGPUData ray)
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

	if (t >= -EPSILON && t < collision[index].distance)
	{
		collision[index].point				= intersectionPoint;
		collision[index].normal				= normalize(face.normal);
		collision[index].tangent			= normalize(v1.tangent);
		collision[index].textCoord			= getTextCoord(intersectionPoint, v1, v2, v3);
		collision[index].distance			= distanceToOrigin;				
		collision[index].angle				= acos(clamp(dot(collision[index].normal, normalize(-ray.direction)), -1.0f, 1.0f));

		return true;
	}

	return false;							// There is a LINE intersection but no RAY intersection
}