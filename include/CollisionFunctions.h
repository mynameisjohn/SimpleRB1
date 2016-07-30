#pragma once

#include "GL_Util.h"

// Forward all these types, it's all pointer based
struct RigidBody2D;
struct Circle;
struct AABB;
struct Plane;
struct Triangle;

vec2 perp( vec2 v );	// returns (-v.y, v.x)
float cross2D( vec2 a, vec2 b );
vec2 maxComp( vec2 v );	// zeroes all but the biggest
vec2 projectOnEdge( vec2 p, vec2 e0, vec2 e1 );

////////////////////////////////////////////////////////////////////////////

// Functions for getting speculative contacts
Contact GetSpecContact( Circle * pA, Circle * pB );
Contact GetSpecContact( Circle * pCirc, AABB * pAABB );
Contact GetSpecContact( Circle * pCirc, Plane * pPlane );

Contact GetSpecContact( AABB * pA, AABB * pB );
Contact GetSpecContact( AABB * pAABB, Plane * pPlane );

Contact GetSpeculativeContact( const RigidBody2D * pA, const RigidBody2D * pB );
Contact GetSpeculativeContact( const Plane * pPlane, const RigidBody2D * pB );

////////////////////////////////////////////////////////////////////////////

// Functions for detecting overlap
bool IsOverlapping( Circle * pA, Circle * pB );
bool IsOverlapping( Circle * pCirc, AABB * pAABB );
bool IsOverlapping( Circle * pCirc, Triangle * pT );

bool IsOverlappingX( AABB * pA, AABB * pB );
bool IsOverlappingY( AABB * pA, AABB * pB );
bool IsOverlapping( AABB * pA, AABB * pB );
bool IsOverlapping( AABB * pAABB, Triangle * pT );

bool IsOverlapping( const Shape * pA, const Shape * pB );

////////////////////////////////////////////////////////////////////////////

// Determine if a point is inside an object
bool IsPointInside( vec2 p, Circle * pCirc );
bool IsPointInside( vec2 p, AABB * pAABB );

////////////////////////////////////////////////////////////////////////////

// Special functions used when working with boxes (they have points)
// Box functions that take an index use the following notation
//			                
//	Verts: 3---0  Normals: --3--
//		   |   |		 <-2   0->
//		   2---1		   --1--
////////////////////////////////////////////////////////////////////////////

// AABB Functions
glm::vec2 GetVert( AABB * pAABB, int idx );	
glm::vec2 GetNormal( AABB * pAABB, int idx );

////////////////////////////////////////////////////////////////////////////

vec2 ClosestPtToTriangle( vec2 vA, vec2 vB, vec2 vC, vec2 p );