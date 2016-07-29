#pragma once

#include "RigidBody2D.h"
#include "Plane.h"
#include "Camera.h"
#include "Shader.h"
#include "Drawable.h"
#include "Contact.h"

#include <vector>
#include <array>

#include <SDL.h>

class Scene
{
public:
	Scene();
	~Scene();

	void Draw();
	void Update();

	void SetQuitFlag( bool bQuit );
	bool GetQuitFlag() const;

	void SetDrawContacts( bool bDrawContacts );
	bool GetDrawContacts() const;

	void SetPauseCollision( bool bPauseCollision );
	bool GetPauseCollision() const;

	const Plane * GetPlane( const size_t planeIdx ) const;
	const Shader * GetShaderPtr() const;
	const Camera * GetCameraPtr() const;
	std::list<Contact *> GetContacts() const;
	const Drawable * GetDrawable( const size_t drIdx ) const;
	const RigidBody2D * GetRigidBody2D( const size_t rbIdx ) const;
	const SoftBody2D * GetSoftBody2D( const size_t sbIdx ) const;

	bool InitDisplay( std::string strWindowName, vec4 v4ClearColor, std::map<std::string, int> mapDisplayAttrs );
	
	int AddDrawableIQM( std::string strIqmFile, vec2 T, vec2 S, vec4 C, float theta = 0.f );
	int AddDrawableTri( std::string strName, std::array<vec3, 3> triVerts, vec2 T, vec2 S, vec4 C, float theta = 0.f );
	int AddSoftBody( Shape::EType eType, glm::vec2 v2Pos, std::map<std::string, float> mapDetails );
	int AddRigidBody(Shape::EType eType, glm::vec2 v2Vel, glm::vec2 v2Pos, float fMass, float fElasticity, std::map<std::string, float> mapDetails );
	int AddCollisionPlane( glm::vec2 N, float d );
private:
	bool m_bQuitFlag;
	bool m_bDrawContacts;
	bool m_bPauseCollision;
	SDL_GLContext m_GLContext;
	SDL_Window * m_pWindow;
	Shader m_Shader;
	Camera m_Camera;
	std::vector<Drawable> m_vDrawables;
	std::vector<SoftBody2D> m_vSoftBodies;
	std::vector<RigidBody2D> m_vRigidBodies;
	std::vector<Plane> m_vCollisionPlanes;
	std::list<Contact> m_liSpeculativeContacts;
	Contact::Solver m_ContactSolver;
};
