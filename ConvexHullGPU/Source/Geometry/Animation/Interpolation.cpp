#include "stdafx.h"
#include "Interpolation.h"


Interpolation::Interpolation(std::vector<vec4>& waypoints) : _waypoints(waypoints)
{
}

Interpolation::~Interpolation()
{
}

void Interpolation::exportPoints()
{
	unsigned index = 1;
	
	for(vec4& point: _waypoints)
	{
		std::cout << "path[" << index++ << "] = vec2(" << point.x << ", " << point.z << ");" << std::endl;
	}
}

float Interpolation::getLineLength(std::vector<vec4>& points)
{
	float length = 0.0f;

	for (unsigned i = 1; i < points.size(); ++i)
	{
		length += glm::distance(points[i], points[i - 1]);
	}

	return length;
}