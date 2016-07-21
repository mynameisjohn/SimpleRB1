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
	eType( EType::None ),
	fMass( 0 ),
	fElast( 0 )
{}

RigidBody2D::RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity ) :
	eType( EType::None ),
	fMass( mass ),
	fElast( elasticity ),
	v2Vel( vel ),
	v2Center( c )
{}

/*static*/ RigidBody2D RigidBody2D::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity )
{
	return RigidBody2D( vel, c, mass, elasticity );
}

////////////////////////////////////////////////////////////////////////////

/*static*/ Contact RigidBody2D::GetSpeculativeContact( const RigidBody2D * pA, const RigidBody2D * pB )
{
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

/*static*/ Contact RigidBody2D::GetSpeculativeContact( const Plane * pPlane, const RigidBody2D * pB )
{
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

void RigidBody2D::UpdateDrawable(Drawable * pDrawable) const
{
	if ( pDrawable )
		pDrawable->SetPos2D( v2Center );
}

// I need a good file for these
vec2 perp( vec2 v )
{
	return vec2( -v.y, v.x );
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