// Computes intersection between a ray and an axis-aligned bounding box. Slabs method
bool rayAABBIntersection(in RayGPUData ray, vec3 minPoint, vec3 maxPoint, float currentMinDistance)
{
	vec3 tMin = (minPoint - ray.origin) / ray.direction;
    vec3 tMax = (maxPoint - ray.origin) / ray.direction;
    vec3 t1 = min(tMin, tMax);
    vec3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);

    return tFar >= tNear;
}