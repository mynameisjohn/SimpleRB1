#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"
#include "Plane.h"

#include <glm/gtx/norm.hpp>

////////////////////////////////////////////////////////////////////////////

float AABB::Width() const
{
	return 2.f * v2HalfDim.x;
}

float AABB::Height() const
{
	return 2.f * v2HalfDim.y;
}

float AABB::Left() const
{
	return v2Center.x - v2HalfDim.x;
}

float AABB::Right() const
{
	return v2Center.x + v2HalfDim.x;
}

float AABB::Top() const
{
	return v2Center.y + v2HalfDim.y;
}

float AABB::Bottom() const
{
	return v2Center.y - v2HalfDim.y;
}

glm::vec2 AABB::HalfDim() const
{
	return v2HalfDim;
}

glm::vec2 AABB::Clamp( const glm::vec2 p ) const
{
	return glm::clamp( p, v2Center - v2HalfDim, v2Center + v2HalfDim );
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 AABB::GetFaceNormalFromPoint( const glm::vec2 p ) const
{
	vec2 n( 0 );

	if ( p.x < Right() && p.x > Left() )
	{
		if ( p.y < Bottom() )
			n = vec2( 0, -1 );
		else
			n = vec2( 0, 1 );
	}
	else
	{
		if ( p.x < Left() )
			n = vec2( -1, 0 );
		else
			n = vec2( 1, 0 );
	}

	return n;
}

////////////////////////////////////////////////////////////////////////////

/*static*/ RigidBody2D AABB::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R )
{
	RigidBody2D ret = RigidBody2D::Create( vel, c, mass, elasticity );
	ret.v2HalfDim = v2R;
	ret.eType = RigidBody2D::EType::AABB;
	return ret;
}

////////////////////////////////////////////////////////////////////////////

/*static*/ RigidBody2D AABB::Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h )
{
	RigidBody2D ret = RigidBody2D::Create( vel, vec2( x, y ), mass, elasticity );
	ret.v2HalfDim = vec2( w, h ) / 2.f;
	ret.eType = RigidBody2D::EType::AABB;
	return ret;
}

////////////////////////////////////////////////////////////////////////////

// This is rather verbose, but it gets the job done
Contact GetSpecContact( AABB * pA, AABB * pB )
{
	// Contact normal and indices from each box
	vec2 n;
	int vIdxA, vIdxB;

	// Get the overlap directions
	bool bX = IsOverlappingX( pA, pB );
	bool bY = IsOverlappingY( pA, pB );

	// If we have a genuine overlap, then I haven't done my job...
	if ( bX && bY )
	{
		// Find the direction of least penetration (X or Y),
		// take A's face in that direction to be the collision normal
		float fPenX( 0 ), fPenY( 0 );
		int vIdxAX( 0 ), vIdxAY( 0 ), vIdxBX( 0 ), vIdxBY( 0 );
		vec2 v2NrmX, v2NrmY;

		// Determine which direction the overlap occurs in X
		if ( pA->Right() > pB->Left() && pA->Left() < pB->Right() )
		{
			// A is overlapping B from the left
			fPenX = std::min( 0.f, pB->Left() - pA->Right() );
			v2NrmX = vec2( 1, 0 );
			vIdxAX = 0;
			vIdxBX = 2;
		}
		else
		{
			// A is overlapping B from the right
			fPenX = std::min( 0.f, pA->Left() - pB->Right() );
			v2NrmX = vec2( -1, 0 );
			vIdxAX = 2;
			vIdxBX = 0;
		}

		// Determine which direction the overlap occurs in X
		if ( pA->Top() > pB->Bottom() && pA->Bottom() < pB->Top() )
		{
			// A is overlapping B from below
			fPenY = std::min( 0.f, pB->Bottom() - pA->Top() );
			v2NrmY = vec2( 0, 1 );
			vIdxAY = 3;
			vIdxBY = 1;
		}
		else
		{
			// A is overlapping B from above
			fPenY = std::min( 0.f, pA->Bottom() - pB->Top() );
			v2NrmY = vec2( 0, -1 );
			vIdxAY = 1;
			vIdxBY = 3;
		}

		// Pick the least penetration distance
		// The penetration distances are <= 0,
		// and we want the one closest to 0
		if ( fPenX > fPenY )
		{
			// X penetrates less than Y
			n = v2NrmX;
			vIdxA = vIdxAX;
			vIdxB = vIdxBX;
		}
		else
		{
			// the other way around
			n = v2NrmY;
			vIdxA = vIdxAY;
			vIdxB = vIdxBY;
		}
	}
	// We're overlapping X, worry about vertical collisions
	else if ( bX )
	{
		// If We're below
		if ( pA->Top() <= pB->Bottom() )
		{
			// top of A, bottpm of B
			n = vec2( 0, 1 );
			vIdxA = 3;
			vIdxB = 1;
		}
		else
		{
			// bottom of A, top of B
			n = vec2( 0, -1 );
			vIdxA = 1;
			vIdxB = 3;
		}
	}
	// We're overlapping Y, worry about horizontal collisions
	else if ( bY )
	{
		if ( pA->Right() < pB->Left() )
		{
			// right of A, left of B
			n = vec2( 1, 0 );
			vIdxA = 0;
			vIdxB = 2;
		}
		else
		{
			// left of A, right of B
			n = vec2( -1, 0 );
			vIdxA = 2;
			vIdxB = 0;
		}
	}
	else
	{
		// If we aren't overlapping in either direction
		// handle a potential corner collision
		bool bAIsLeft = (pA->Right() < pB->Left());
		bool bAIsBelow = (pA->Top() < pB->Bottom());
		if ( bAIsLeft )
		{
			if ( bAIsBelow )
			{
				// Top right of A, bottom left of B
				vIdxA = 0;
				vIdxB = 2;
			}
			else
			{
				// Bottom right of A, top left of B
				vIdxA = 1;
				vIdxB = 3;
			}
		}
		else
		{
			if ( bAIsBelow )
			{
				// Top left of A, bottom right of B
				vIdxA = 3;
				vIdxB = 1;
			}
			else
			{
				// Bottom left of A, top right of B
				vIdxA = 2;
				vIdxB = 0;
			}
		}

		// We don't need to average contact positions for the corner case
		vec2 posA = GetVert( pA, vIdxA );
		vec2 posB = GetVert( pB, vIdxB );
		n = glm::normalize( posB - posA );
		float fDist = glm::distance( posA, posB );
		return Contact( pA, pB, posA, posB, n, fDist );
	}

	// For the face case, we get the two vertices from each colliding face
	// and average them per face, so that the collision is not diminished
	// by the radius arm of the contact. Distance is along direction of normal
	vec2 posA = 0.5f * (GetVert( pA, vIdxA ) + GetVert( pA, vIdxA + 1 ));
	vec2 posB = 0.5f * (GetVert( pB, vIdxB ) + GetVert( pB, vIdxB + 1 ));
	float fDist = glm::dot( n, posB - posA );
	return Contact( pA, pB, posA, posB, n, fDist );
}

////////////////////////////////////////////////////////////////////////////

Contact GetSpecContact( AABB * pAABB, Plane * pPlane )
{
	float fDistToPlane = glm::dot( pAABB->v2Center, pPlane->v2Normal ) - pPlane->fDist;
	glm::vec2 posA = pAABB->v2Center - fDistToPlane * pPlane->v2Normal;
	glm::vec2 posB = pAABB->Clamp( posA );
	float fDist = glm::dot( pPlane->v2Normal, posB - posA );
	return Contact( pPlane, pAABB, posA, posB, pPlane->v2Normal, fDist );
}

////////////////////////////////////////////////////////////////////////////

bool IsPointInside( vec2 p, AABB * pAABB )
{
	bool bX = fabs( p.x - pAABB->v2Center.x ) < pAABB->v2HalfDim.x;
	bool bY = fabs( p.y - pAABB->v2Center.y ) < pAABB->v2HalfDim.y;
	return bX && bY;
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlappingX( AABB* pA, AABB* pB )
{
	return (pA->Right() < pB->Left() || pA->Left() > pB->Right()) == false;
}

bool IsOverlappingY( AABB* pA, AABB* pB )
{
	return (pA->Top() < pB->Bottom() || pA->Bottom() > pB->Top()) == false;
}

bool IsOverlapping( AABB* pA, AABB* pB )
{
	return IsOverlappingX( pA, pB ) && IsOverlappingY( pA, pB );
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetVert( AABB * pAABB, int idx )
{
	vec2 ret( 0 );
	vec2& R = pAABB->v2HalfDim;			// 3---0
	switch ( (idx + 4) % 4 )			// |   |
	{									// 2---1
		case 0:
			return pAABB->v2Center + R;
		case 1:
			return pAABB->v2Center + vec2( R.x, -R.y );
		case 2:
			return pAABB->v2Center - R;
		case 3:
		default:
			return pAABB->v2Center + vec2( -R.x, R.y );
	}
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetNormal( AABB * pAABB, int idx )
{										// --0--
	switch ( (idx + 4) % 4 )			// 3   1
	{									// --2--
		case 0:
			return vec2( 1, 0 );
		case 1:
			return vec2( 0, -1 );
		case 2:
			return vec2( -1, 0 );
		case 3:
		default:
			return vec2( 0, 1 );
	}
}
