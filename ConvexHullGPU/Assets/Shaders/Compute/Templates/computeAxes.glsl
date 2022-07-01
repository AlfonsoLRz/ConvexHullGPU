void getRadiusAxes(vec3 n, vec3 up, out vec3 u, out vec3 v)
{
	u = normalize(cross(up, n));
	v = normalize(cross(n, u));
}