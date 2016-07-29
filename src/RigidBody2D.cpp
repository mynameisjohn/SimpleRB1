#include "RigidBody2D.h"
#include "GL_Util.h"
#include "Util.h"
#include "CollisionFunctions.h"
#include "Drawable.h"

#include <glm/gtx/norm.hpp>

// Euler integrate rigid body translation/rotation
void EulerAdvance( RigidBody2D * pRB, float fDT )
{
	vec2 v2Accel = pRB->v2Force / pRB->fMass;
	pRB->v2Center += fDT * pRB->v2Vel;
	pRB->v2Vel += fDT * v2Accel;
}

void VerletAdvance( RigidBody2D * pRB, float fDT )
{
	vec2 v2Accel = pRB->v2Force / pRB->fMass;
	pRB->v2Center += fDT * ( pRB->v2Vel + 0.5f * fDT * v2Accel );
	pRB->v2Vel += fDT * v2Accel;
}

void RigidBody2D::Integrate(float fDT)
{
	VerletAdvance( this, fDT );
	v2Force = vec2();
}

RigidBody2D::RigidBody2D() :
	Shape(),
	fMass( 0 ),
	fElast( 0 )
{}

Shape::Shape() :
	eType( EType::None )
{}

vec2 Shape::Position() const
{
	return v2Center;
}

Shape::EType Shape::Type() const
{
	return eType;
}

RigidBody2D::RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity ) :
	Shape(),
	fMass( mass ),
	fElast( elasticity ),
	v2Vel( vel )
{
	v2Center = c;
}

///*static*/ RigidBody2D RigidBody2D::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity )
//{
//	return RigidBody2D( vel, c, mass, elasticity );
//}

////////////////////////////////////////////////////////////////////////////

/*static*/ Contact GetSpeculativeContact( const RigidBody2D * pA, const RigidBody2D * pB )
{
	using EType = RigidBody2D::EType;
	switch ( pA->eType )
	{
		case EType::Circle:
		{
			Circle * pCircA = (Circle *) pA;
			switch ( pB->eType )
			{
				case EType::Circle:
					return GetSpecContact( pCircA, (Circle *) pB );
				case EType::AABB:
					return GetSpecContact( pCircA, (AABB *) pB );
				default:
					break;
			}
			break;
		}
		case EType::AABB:
		{
			AABB * pBoxA = (AABB *) pA;
			switch ( pB->eType )
			{
				case EType::Circle:
					return GetSpecContact( (Circle *) pB, pBoxA );
				case EType::AABB:
					return GetSpecContact( pBoxA, (AABB *) pB );
				default:
					break;
			}
			break;
		}
	}

	throw std::runtime_error( "Error: Invalid rigid body type!" );
	return Contact();
}

////////////////////////////////////////////////////////////////////////////

/*static*/ Contact GetSpeculativeContact( const Plane * pPlane, const RigidBody2D * pB )
{
	using EType = RigidBody2D::EType;
	switch ( pB->eType )
	{
		case EType::AABB:
			return GetSpecContact( (AABB *) pB, (Plane *) pPlane );
		case EType::Circle:
			return GetSpecContact( (Circle *) pB, (Plane *) pPlane );
		default:
			break;
	}

	throw std::runtime_error( "Error: Invalid rigid body type!" );
	return Contact();
}

////////////////////////////////////////////////////////////////////////////

vec2 RigidBody2D::GetMomentum() const
{
	return fMass * v2Vel;
}

float RigidBody2D::GetKineticEnergy() const
{
	return 0.5f * fMass * glm::dot( v2Vel, v2Vel );
}

//void RigidBody2D::UpdateDrawable(Drawable * pDrawable) const
//{
//	if ( pDrawable )
//		pDrawable->SetPos2D( v2Center );
//}

// I need a good file for these
vec2 perp( vec2 v )
{
	return vec2( -v.y, v.x );
}

float cross2D( vec2 a, vec2 b )
{
	return a.x*b.y - a.y*b.x;
}

vec2 maxComp( vec2 v )
{
	if ( feq( v.x, v.y ) )
		return v;
	else if ( fabs( v.x ) > fabs( v.y ) )
		return vec2( v.x, 0.f );
	else
		return vec2( 0.f, v.y );
}

bool feq( float a, float b, float diff )
{
	return fabs( a - b ) < diff;
}

// Static function to project a point p along the edge
// between points e0 and e1
vec2 projectOnEdge( vec2 p, vec2 e0, vec2 e1 )
{
	// u and v form a little triangle
	vec2 u = p - e0;
	vec2 v = e1 - e0;

	// find the projection of u onto v, parameterize it
	float t = glm::dot( u, v ) / glm::dot( v, v );

	//std::cout << t << ", " << glm::dot(u, v) << ", " << glm::dot(v,v) << std::endl;

	// Clamp between two edge points
	return e0 + v * clamp( t, 0.f, 1.f );
}

vec2 ClosestPtToTriangle( const Triangle * const pT, vec2 p )
{
	// Face edges
	vec2 ab = pT->v2B - pT->v2A, ac = pT->v2C - pT->v2A, bc = pT->v2C - pT->v2B;

	float s_ab = glm::dot( p - pT->v2A, ab );		// unnormalized along ab
	float s_ba = glm::dot( p - pT->v2B, -ab );	// unnormalized along ba

	float t_bc = glm::dot( p - pT->v2B, bc );		// unnormalized along bc
	float t_cb = glm::dot( p - pT->v2C, -bc );	// and along cb

	float u_ac = glm::dot( p - pT->v2A, ac );		// along ac
	float u_ca = glm::dot( p - pT->v2C, -ac );	// along ca

	// If the unnormalized param from a to b
	// and from a to c is negative, a is closest
	if ( s_ab <= 0 && u_ac <= 0 )
		return pT->v2A;

	// If the unnormalized param from b to a
	// and from b to c is negative, b is closest
	if ( s_ba <= 0 && t_bc <= 0 )
		return pT->v2B;

	// If the unnormalized param from c to a
	// and from c to b is negative, c is closest
	if ( u_ca <= 0 && t_cb <= 0 )
		return pT->v2C;

	// If it wasn't one of those, check the edges
	// For each face edge, create a new triangle
	// with p as one of the verts and find its
	// signed area (positive half determined by n)
	float n = cross2D( ab, ac );

	// If proj(p, AB) is between A and B (both params positive)
	// check the signed area of pab, return proj if negative
	if ( s_ab > 0 && s_ba > 0 )
	{
		float sA_ab = n * cross2D( pT->v2A - p, pT->v2B - p );
		if ( sA_ab <= 0 )
		{
			float s = s_ab / (s_ab + s_ba);
			return pT->v2A + s * ab;
		}
	}

	// BC
	if ( t_bc > 0 && t_cb > 0 )
	{
		float sA_bc = n * cross2D( pT->v2B - p, pT->v2C - p );
		if ( sA_bc <= 0 )
		{
			float t = t_bc / (t_bc + t_cb);
			return pT->v2B + t * bc;
		}
	}

	// CA
	if ( u_ac > 0 && u_ca > 0 )
	{
		float sA_ca = n * cross2D( pT->v2C - p, pT->v2A - p );
		if ( sA_ca <= 0 )
		{
			float u = u_ac / (u_ac + u_ca);
			return pT->v2C + u * ac;
		}
	}

	// Inside triangle, return p
	return p;
}