#pragma once

#include <glm/vec2.hpp>

struct Plane
{
	glm::vec2 v2Normal;
	float fDist;

	void SetNormal( glm::vec2 v2N )
	{
		v2Normal = v2N;
	}
	void SetDist( float fD )
	{
		fDist = fD;
	}
};
