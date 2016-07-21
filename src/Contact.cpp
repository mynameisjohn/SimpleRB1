#include "Contact.h"
#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"
#include "Drawable.h"
#include <glm/vec4.hpp>
#include <glm/gtc/random.hpp>

#include <iostream>

// Called from constructor, used to get inverse mass
// (used to calculate inertia when rotation was a thing)
void Contact::init()
{
	// How should I handle this?
	if ( m_bHasPlane == false && m_pA == m_pB )
		throw std::runtime_error( "Error: Contact created from one object!" );

	// Find the inverse denom
	// I'm not sure if this is correct without rotation
	float fDenom( 0.f );
	if ( m_bHasPlane )
	{
		// Plane contacts only have one object with mass
		fDenom = 1.f / m_pB->fMass;
	}
	else
	{
		// Sum denominator
		for ( RigidBody2D * pRB : { m_pA, m_pB } )
		{
			// The inverse mass denominator is a coeffecient used to calculate impulses
			fDenom += 1.f / pRB->fMass;
		}
	}

	// Is this necessary?
	if ( fDenom < kEPS )
		throw std::runtime_error( "Error: Invalid inertial denominator calculated!" );

	// Set the inverse mass value
	m_fInvMass = 1.f / fDenom;
}

// Default ctor
Contact::Contact() :
	m_bHasPlane( false ),
	m_bIsColliding( false ),
	m_pA( nullptr ),
	m_pB( nullptr ),
	m_v2Pos{ vec2(), vec2() },
	m_v2Normal( vec2() ),
	m_fDist( 0 ),
	m_fCurImpulse( 0 ),
	m_fInvMass(0)
{}

// RB-RB contact ctor
Contact::Contact( RigidBody2D * pA, RigidBody2D * pB, const vec2 posA, const vec2 posB, const vec2 nrm, const float d ) :
	m_bHasPlane( false ),
	m_bIsColliding( false ),
	m_pA( pA ),
	m_pB( pB ),
	m_v2Pos{ posA, posB },
	m_v2Normal( nrm ),
	m_fDist( d ),
	m_fCurImpulse( 0 )
{
	init();
}

// Plane-RB ctor
Contact::Contact( Plane * pPlaneA, RigidBody2D * pB, const vec2 posA, const vec2 posB, const vec2 nrm, const float d ) :
	m_bHasPlane( true ),
	m_bIsColliding( false ),
	m_pPlaneA( pPlaneA ),
	m_pB( pB ),
	m_v2Pos{ posA, posB },
	m_v2Normal( nrm ),
	m_fDist( d ),
	m_fCurImpulse( 0 )
{
	init();
}

// Apply a collision impulse to the two contact objects
void Contact::ApplyImpulse( float fMag )
{
	// Calculate the new impulse and apply the change
	float newImpulse = std::min( 0.f, fMag + m_fCurImpulse );
	float delImpulse = newImpulse - m_fCurImpulse;

	// Find the direction along our collision normal
	vec2 v2Impulse = delImpulse * m_v2Normal;

	// If no plane and A is not immovable, jostle A
	if ( m_bHasPlane == false && m_pA->fMass > 0 )
	{
		m_pA->v2Vel += v2Impulse / m_pA->fMass;
	}

	// Jostle B
	if ( m_pB->fMass > 0 )
		m_pB->v2Vel -= v2Impulse / m_pB->fMass;

	// Add impulse to member var
	m_fCurImpulse = newImpulse;
}

// Various gets
vec2 Contact::GetNormal() const
{
	return m_v2Normal;
}

float Contact::GetDistance() const
{
	return m_fDist;
}

// Planes should have their own coefs, I guess
float Contact::GetAvgCoefRest() const
{
	if ( m_bHasPlane )
		return m_pB->fElast;
	else
		return .5f * (m_pA->fElast + m_pB->fElast);
}

float Contact::GetInvMass() const
{
	return m_fInvMass;
}

float Contact::GetRelVel() const
{
	// If plane, vel is B's along plane normal
	if ( m_bHasPlane )
	{
		return glm::dot( m_v2Normal, m_pB->v2Vel );
	}

	// Otherwise it's vel of system along contact normal
	const float vA_N = glm::dot( m_v2Normal, m_pA->v2Vel );
	const float vB_N = glm::dot( m_v2Normal, m_pB->v2Vel );
	return vB_N - vA_N;
}

bool Contact::HasPlane() const
{
	return m_bHasPlane;
}

float Contact::GetCurImpulse() const
{
	return m_fCurImpulse;
}

const Plane * Contact::GetPlane() const
{
	return m_pPlaneA;
}

const RigidBody2D * Contact::GetBodyA() const
{
	return m_pA;
}

const RigidBody2D * Contact::GetBodyB() const
{
	return m_pB;
}

bool Contact::IsColliding() const
{
	return m_bIsColliding;
}

void Contact::setIsColliding( bool bColliding )
{
	m_bIsColliding = bColliding;
}

// Contact Solver
Contact::Solver::Solver():
	m_nIterations(1)
{}

Contact::Solver::Solver( uint32_t nIterations ) :
	m_nIterations( nIterations )
{}

uint32_t Contact::Solver::Solve( std::list<Contact>& liContacts )
{
	// Return the # of collisions
	uint32_t uNumCollisions( 0 );

	// Iterate and solve contacts
	for ( uint32_t nIt = 0; nIt < m_nIterations; nIt++ )
	{
		// The # of collisions this iteration
		uint32_t uColCount = 0;

		// Walk the contacts
		for ( Contact& c : liContacts )
		{
			// Coeffcicient of restitution, plus 1
			const float fCr_1 = 1.f + c.GetAvgCoefRest();

			// Get the relative velocity of each body along the contact normal
			float fRelVN = c.GetRelVel();

			// Determine how much velocity we'd need to remove such that
			// in the next iteration the two objects will be touching
			float fVelNeeded = c.GetDistance() * g_fInvTimeStep;
			float fVelToRemove = fRelVN + fVelNeeded;

			// If this is very low
			if ( fVelToRemove < kEPS )
			{
				// apply a collison along the normal
				c.ApplyImpulse( fCr_1 * fRelVN * c.GetInvMass() );
				// Increase collision counter, flag contact as colliding
				uColCount++;
				c.setIsColliding( true );
			}
		}

		// Maybe break if no contacts are colliding
		if ( uColCount == 0 )
			break;
		// Otherwise keep going
		else
			uNumCollisions += uColCount;
	}

	return uNumCollisions;
}

// Used for debugging
void Contact::InitDrawable( std::array<Drawable *, 2> drPtrArr ) const
{
	const float drScale = 0.2f;
	vec4 v4Color = glm::linearRand( vec4( vec3( 0.3f ), 1 ), vec4( 1 ) );
	for ( size_t i = 0; i < 2; i++ )
	{
		if ( drPtrArr[i] )
		{
			quatvec qv; 
			qv.vec = vec3( m_v2Pos[i], 1.f );
			drPtrArr[i]->Init( "../models/quad.iqm", v4Color, qv, vec2( drScale ) );
		}
	}
}