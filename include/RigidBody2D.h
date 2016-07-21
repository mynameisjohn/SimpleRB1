#pragma once

#include "Contact.h"
#include "quatvec.h"
#include "EntComponent.h"

#include <glm/mat2x2.hpp>
#include <glm/vec2.hpp>

class Drawable;
struct Plane;

// Let's see if I can make this a POD union
struct RigidBody2D : public EntComponent
{
	// All rigid bodies are the same type,
	// but this enum dictates what they really are
	enum class EType : int
	{
		None,
		Circle,
		AABB
	};

	EType eType;		// Primitive type
	float fMass;		// Mass
	float fElast;		// Elasticity
	glm::vec2 v2Center;	// Center position
	glm::vec2 v2Vel;	// Velocity
	glm::vec2 v2Force;	// Acting force

	// The big union
	union
	{
		struct { glm::vec2 v2HalfDim; };	// Box Data
		struct { float fRadius; };			// Circle Data
		
		// For planes - there's no reason a plane has to have
		// all the stuff a rigidbody has, but it's convenient for now
		struct
		{
			glm::vec2 v2Normal;
			float fDist;
		};
	};

	// Default constructor
	RigidBody2D();

	// Various gets
	glm::vec2 GetMomentum() const;
	float GetKineticEnergy() const;

	void UpdateDrawable( Drawable * pDrawable ) const;
	
	// Advance the object's position and velocity
	// v2Force determines dV and is zeroed in this function
	void Integrate( float fDT );

	// Until we get rid of this dumb list pattern. This is handled internally via a switch
	static Contact GetSpeculativeContact( const RigidBody2D * pA, const RigidBody2D * pB );
	static Contact GetSpeculativeContact( const Plane * pPlane, const RigidBody2D * pB );

	// Interesting constructor is protected, called
	// by class static methods from child classes (?)
protected:
	RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity);
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity);
};

// I'm not really sure about this...
// But as long as these subclasses declare
// no internal members I think we're good
struct Circle : public RigidBody2D
{
	Circle() = delete;
	float Radius() const;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float radius );
};

struct AABB : public RigidBody2D
{
	AABB() = delete;

	// useful things
	float Width() const;
	float Height() const;
	float Left() const;
	float Right() const;
	float Top() const;
	float Bottom() const;
	glm::vec2 Clamp( const glm::vec2 p ) const;
	glm::vec2 GetFaceNormalFromPoint( const glm::vec2 p ) const;
	glm::vec2 HalfDim() const;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R );
	static RigidBody2D Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h );
};