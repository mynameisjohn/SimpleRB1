#pragma once

// Forward for RigidBody2D
struct RigidBody2D;
struct Plane;

// Forward for debugging
class Drawable;

#include <glm/vec2.hpp>
#include <list>
#include <array>

// Contact for speculative contact collision detection
class Contact
{
public:
	// Construct from pairs (I wonder if I can just keep them in memory)
	Contact();
	Contact( RigidBody2D * pA, RigidBody2D * pB,		// Pointers to the pair
			 const glm::vec2 posA, const glm::vec2 posB,// Positions of the pair
			 const glm::vec2 nrm,						// Collision normal
			 const float d );							// Distance

	Contact( Plane * pA, RigidBody2D * pB,		// Pointers to the pair
			 const glm::vec2 posA, const glm::vec2 posB,// Positions of the pair
			 const glm::vec2 nrm,						// Collision normal
			 const float d );							// Distance

	// Apply some collision impulse
	void ApplyImpulse( float fMag );

	float GetDistance() const;
	float GetAvgCoefRest() const;
	float GetInvMass() const;
	float GetCurImpulse() const;

	bool HasPlane() const;
	glm::vec2 GetNormal() const;
	float GetRelVel() const;
	const Plane * GetPlane() const;
	const RigidBody2D * GetBodyA() const;
	const RigidBody2D * GetBodyB() const;

	// The Solver class, which really only does one thing...
	class Solver
	{
	public:
		Solver();
		Solver( uint32_t nIterations );
		uint32_t Solve( std::list<Contact>& liContacts );
	private:
		uint32_t m_nIterations;
	};

	// Init a drawable, for debugging purposes
	void InitDrawable( std::array<Drawable *, 2> drPtrArr ) const;

	bool IsColliding() const;
	class Solver;
protected:
	void setIsColliding( bool bCollidng );

private:
	// Some convenient unions
	union
	{
		Plane * m_pPlaneA;
		RigidBody2D * m_pA;
	};
	RigidBody2D * m_pB;
	union
	{	// The positions of the contact points
		std::array<glm::vec2, 2> m_v2Pos;
		struct { glm::vec2 m_v2PosA; glm::vec2 m_v2PosB; };
	};

	bool m_bHasPlane;		// duh
	bool m_bIsColliding;	// Whether or not the contact is colliding
	float m_fDist;			// The distance between the contact pair
	float m_fInvMass;		// 1 / the impulse mass
	float m_fCurImpulse;	// The accumulated impulse value
	glm::vec2 m_v2Normal;	// The collision normal (out of A)

	void init();
};