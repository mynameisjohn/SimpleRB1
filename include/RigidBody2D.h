#pragma once

#include "Contact.h"
#include "quatvec.h"
#include "EntComponent.h"

#include <glm/mat2x2.hpp>
#include <glm/vec2.hpp>

// Why make this weird class?
// So I can keep all types of shapes
// inside the contiguous data structures
struct Shape : public EntComponent
{
	// Enum meant to represent the type of shape
	// (or rather, which union members mean something)
	enum class EType : int
	{
		None,
		Circle,
		AABB,
		Triangle
	};

	bool bActive;		// If the shape is in the mix
	EType eType;		// Primitive type
	glm::vec2 v2Center;	// Center position

	// The big union
	union
	{
		struct { float fRadius; };			// Circle Data
		struct { glm::vec2 v2HalfDim; };	// Box Data
		struct { glm::vec2 v2A, v2B, v2C; };// Triangle Data
	};

	Shape();
	Shape( glm::vec2 v2C );

	void SetIsActive( bool b );
	bool GetIsActive() const;

	glm::vec2 Position() const;
	EType Type() const;
	void SetCenterPos( glm::vec2 v2Pos );
};

// Useful typedef
using SoftBody2D = Shape;

// RigidBody2D does everything a 
// soft body does along with mass,
// elasticity, velocity, and acting force
struct RigidBody2D : public SoftBody2D
{
	RigidBody2D();

	float fMass;		// Mass
	float fElast;		// Elasticity
	glm::vec2 v2Vel;	// Velocity
	glm::vec2 v2Force;	// Acting force

	// Various gets
	glm::vec2 GetMomentum() const;
	float GetKineticEnergy() const;

	// Advance the object's position and velocity
	// v2Force determines dV and is zeroed in this function
	void Integrate( float fDT );


	RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity);
};

// General pattern here is
//		disabled default constructor
//		static factory methods that set prim type
struct Circle : public Shape
{
	Circle() = delete;

	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float fRadius );
	static SoftBody2D Create( glm::vec2 c, float fRadius );

	float Radius() const;
};

struct AABB : public Shape
{
	AABB() = delete;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R );
	static RigidBody2D Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h );

	// Static creation function, returns a RigidBody2D
	static SoftBody2D Create( glm::vec2 c, glm::vec2 v2R );
	static SoftBody2D Create( float x, float y, float w, float h );

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
};

struct Triangle : public Shape
{
	Triangle() = delete;

	static SoftBody2D Create( glm::vec2 c, glm::vec2 A, glm::vec2 B, glm::vec2 C );

	float Left() const;
	float Right() const;
	float Top() const;
	float Bottom() const;
	std::array<glm::vec2, 3> Verts() const;
	std::array<glm::vec2, 3> Edges() const;
};