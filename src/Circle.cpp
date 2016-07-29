
#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"
#include "Plane.h"

#include <glm/gtx/norm.hpp>

////////////////////////////////////////////////////////////////////////////

/*static*/ RigidBody2D Circle::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float radius )
{
	RigidBody2D ret( vel, c, mass, elasticity );
	ret.fRadius = radius;
	ret.eType = RigidBody2D::EType::Circle;
	return ret;
}

////////////////////////////////////////////////////////////////////////////

float Circle::Radius() const
{
	return fRadius;
}

////////////////////////////////////////////////////////////////////////////

Contact GetSpecContact( Circle * pA, Circle * pB )
{
	// find and normalize distance
	vec2 d = pB->v2Center - pA->v2Center;
	vec2 n = glm::normalize( d );

	// contact points along circumference
	vec2 a_pos = pA->v2Center + n * pA->Radius();
	vec2 b_pos = pB->v2Center - n * pB->Radius();

	// distance between circumferences
	float dist = glm::length( a_pos - b_pos );

	if ( IsOverlapping( pA, pB ) )
	{
		dist = -dist;
		// std::cout << "Negating circle dist" << std::endl;
	}

	// Construct and return
	return Contact( (RigidBody2D *) pA, (RigidBody2D *) pB, a_pos, b_pos, n, dist );
}

////////////////////////////////////////////////////////////////////////////

// Simlar to the AABB case, but we only care about the center of the circle
Contact GetSpecContact( Circle * pCirc, AABB *  pAABB )
{
	// Determine which feature region we're on
	vec2 n;
	int vIdx( -1 );

	// top/bottom face region - Does the box's width enclose the center?
	if ( pAABB->Right() > pCirc->v2Center.x && pAABB->Left() < pCirc->v2Center.x )
	{
		// Is the circle above the box?
		if ( pCirc->v2Center.y < pAABB->Bottom() )
		{
			vIdx = 1;
			n = vec2( 0, 1 );
		}
		else
		{
			vIdx = 3;
			n = vec2( 0, -1 );
		}
	}
	// left/right face region - Does the box's height enclose the center?
	else if ( pAABB->Top() > pCirc->v2Center.y && pAABB->Bottom() < pCirc->v2Center.y)
	{
		// Is the circle to the left of the box?
		if ( pCirc->v2Center.x < pAABB->Left() )
		{
			vIdx = 2;
			n = vec2( 1, 0 );
		}
		else
		{
			vIdx = 0;
			n = vec2( -1, 0 );
		}
	}
	// Vertex region
	else
	{
		// Determine which vertex
		bool bAIsLeft = (pCirc->v2Center.x < pAABB->Left());
		bool bAIsBelow = (pCirc->v2Center.y < pAABB->Bottom());
		if ( bAIsLeft )
		{
			// Bottom left
			if ( bAIsBelow )
				vIdx = 2;
			// Top left
			else
				vIdx = 3;
		}
		else
		{
			// Bottom right
			if ( bAIsBelow )
				vIdx = 1;
			// Top right
			else
				vIdx = 0;
		}

		// We don't need to average contact positions for the corner case
		vec2 posB = GetVert( pAABB, vIdx );
		n = glm::normalize( posB - pCirc->v2Center );
		vec2 posA = pCirc->v2Center + pCirc->fRadius * n;
		float fDist = glm::dot( posB - posA, n );
		return Contact( (RigidBody2D *) pCirc, (RigidBody2D *) pAABB, posA, posB, n, fDist );
	}

	// For a face region collision, take box position as center of face
	vec2 posB = 0.5f*(GetVert( pAABB, vIdx ) + GetVert( pAABB, vIdx + 1 ));
	vec2 posA = pCirc->v2Center + pCirc->fRadius * n;
	float fDist = glm::dot( posB - posA, n );
	return Contact( (RigidBody2D *) pCirc, (RigidBody2D *) pAABB, posA, posB, n, fDist );
}

////////////////////////////////////////////////////////////////////////////

Contact GetSpecContact( Circle * pCirc, Plane * pPlane )
{
	float fDistToPlane = glm::dot( pCirc->v2Center, pPlane->v2Normal ) - pPlane->fDist;
	glm::vec2 posA = pCirc->v2Center - fDistToPlane * pPlane->v2Normal;
	glm::vec2 posB = pCirc->v2Center - pPlane->v2Normal * pCirc->fRadius;
	return Contact( pPlane, (RigidBody2D *) pCirc, posA, posB, pPlane->v2Normal, fDistToPlane - pCirc->fRadius );
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( Circle * pA, Circle * pB )
{
	float dist = glm::length( pA->v2Center - pB->v2Center );
	float totalRadius = pA->Radius() + pB->Radius();
	return (dist < totalRadius);
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( Circle * pCirc, AABB * pAABB )
{
	float r = pCirc->Radius();
	glm::vec2& C = pCirc->v2Center;
	bool bX = (pAABB->Left() > C.x + r) || (pAABB->Right() < C.y - r) == false;
	bool bY = (pAABB->Bottom() > C.y + r) || (pAABB->Top() < C.y - r);
	return bX && bY;
}


////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( Circle * pCirc, Triangle * pT )
{
	vec2 p = ClosestPtToTriangle( pT, pCirc->v2Center );
	return glm::distance2( pCirc->v2Center, p ) <= powf( pCirc->fRadius, 2 );
}

////////////////////////////////////////////////////////////////////////////

bool IsPointInside( vec2 p, Circle * pCirc )
{
	return glm::length2( pCirc->v2Center - p ) < powf( pCirc->fRadius, 2 );
}