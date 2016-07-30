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

Shape::Shape( glm::vec2 v2C ) :
	eType( EType::None ),
	v2Center( v2C )
{}

void Shape::SetCenterPos( glm::vec2 v2Pos )
{
	v2Center = v2Pos;
}

vec2 Shape::Position() const
{
	return v2Center;
}

Shape::EType Shape::Type() const
{
	return eType;
}

RigidBody2D::RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity ) :
	Shape( c ), 
	fMass( mass ),
	fElast( elasticity ),
	v2Vel( vel )
{}

SoftBody2D Triangle::Create( glm::vec2 c, glm::vec2 A, glm::vec2 B, glm::vec2 C )
{
	SoftBody2D ret( c );
	ret.eType = EType::Triangle;
	ret.v2A = A;
	ret.v2B = B;
	ret.v2C = C;
	return ret;
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

/*static*/ bool IsOverlapping( const Shape * pA, const Shape * pB )
{
	using EType = Shape::EType;
	switch ( pA->eType )
	{
		case EType::Circle:
		{
			Circle * pCircA = (Circle *) pA;
			switch ( pB->eType )
			{
				case EType::Circle:
					return IsOverlapping( pCircA, (Circle *) pB );
				case EType::AABB:
					return IsOverlapping( pCircA, (AABB *) pB );
				case EType::Triangle:
					return IsOverlapping( pCircA, (Triangle *) pB );
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
					return IsOverlapping( (Circle *) pB, pBoxA );
				case EType::AABB:
					return IsOverlapping( pBoxA, (AABB *) pB );
				case EType::Triangle:
					return IsOverlapping( pBoxA, (Triangle *) pB );
				default:
					break;
			}
			break;
		}
		case EType::Triangle:
		{
			Triangle * pTriA = (Triangle *) pA;
			switch ( pB->eType )
			{
				case EType::Circle:
					return IsOverlapping( (Circle *) pB, pTriA );
				case EType::AABB:
					return IsOverlapping( (AABB *) pB, pTriA );
				default:
					break;
			}
			break;
		}
	}

	throw std::runtime_error( "Error: Invalid rigid body type!" );
	return false;
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

vec2 ClosestPtToTriangle( vec2 vA, vec2 vB, vec2 vC, vec2 p )
{
	// Face edges
	vec2 ab = vB - vA, ac = vC - vA, bc = vC - vB;

	float s_ab = glm::dot( p - vA, ab );		// unnormalized along ab
	float s_ba = glm::dot( p - vB, -ab );	// unnormalized along ba

	float t_bc = glm::dot( p - vB, bc );		// unnormalized along bc
	float t_cb = glm::dot( p - vC, -bc );	// and along cb

	float u_ac = glm::dot( p - vA, ac );		// along ac
	float u_ca = glm::dot( p - vC, -ac );	// along ca

	// If the unnormalized param from a to b
	// and from a to c is negative, a is closest
	if ( s_ab <= 0 && u_ac <= 0 )
		return vA;

	// If the unnormalized param from b to a
	// and from b to c is negative, b is closest
	if ( s_ba <= 0 && t_bc <= 0 )
		return vB;

	// If the unnormalized param from c to a
	// and from c to b is negative, c is closest
	if ( u_ca <= 0 && t_cb <= 0 )
		return vC;

	// If it wasn't one of those, check the edges
	// For each face edge, create a new triangle
	// with p as one of the verts and find its
	// signed area (positive half determined by n)
	float n = cross2D( ab, ac );

	// If proj(p, AB) is between A and B (both params positive)
	// check the signed area of pab, return proj if negative
	if ( s_ab > 0 && s_ba > 0 )
	{
		float sA_ab = n * cross2D( vA - p, vB - p );
		if ( sA_ab <= 0 )
		{
			float s = s_ab / (s_ab + s_ba);
			return vA + s * ab;
		}
	}

	// BC
	if ( t_bc > 0 && t_cb > 0 )
	{
		float sA_bc = n * cross2D( vB - p, vC - p );
		if ( sA_bc <= 0 )
		{
			float t = t_bc / (t_bc + t_cb);
			return vB + t * bc;
		}
	}

	// CA (note that ac goes from a to c, so we go from v2A)
	if ( u_ac > 0 && u_ca > 0 )
	{
		float sA_ca = n * cross2D( vC - p, vA - p );
		if ( sA_ca <= 0 )
		{
			float u = u_ac / (u_ac + u_ca);
			return vA + u * ac;
		}
	}

	// Inside triangle, return p
	return p;
}

float Triangle::Left() const
{
	return std::min( { v2A.x, v2B.x, v2C.x } ) + v2Center.x;
}

float Triangle::Right() const
{
	return std::max( { v2A.x, v2B.x, v2C.x } ) + v2Center.x;
}

float Triangle::Bottom() const
{
	return std::min( { v2A.y, v2B.y, v2C.y } ) + v2Center.y;
}

float Triangle::Top() const
{
	return std::max( { v2A.y, v2B.y, v2C.y } ) + v2Center.y;
}

std::array<glm::vec2, 3> Triangle::Verts() const
{
	return{ v2A + v2Center, v2B + v2Center, v2C + v2Center };
}

std::array<glm::vec2, 3> Triangle::Edges() const
{
	return{ v2B - v2A, v2C - v2B, v2A - v2C,   };
}