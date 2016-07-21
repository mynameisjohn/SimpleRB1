#include "Scene.h"
#include "Util.h"

#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

Scene::Scene() :
	m_bQuitFlag( false ),
	m_bDrawContacts( false ),
	m_bPauseCollision( false ),
	m_GLContext( nullptr ),
	m_pWindow( nullptr )
{}

Scene::~Scene()
{
	if ( m_pWindow )
	{
		SDL_DestroyWindow( m_pWindow );
		m_pWindow = nullptr;
	}
	if ( m_GLContext )
	{
		SDL_GL_DeleteContext( m_GLContext );
		m_GLContext = nullptr;
	}
}

void Scene::Draw()
{
	// Clear the screen
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// Bind the shader
	auto sBind = m_Shader.ScopeBind();

	// Get the camera mat as well as some handles
	GLuint pmvHandle = m_Shader.GetHandle( "u_PMV" );
	GLuint clrHandle = m_Shader.GetHandle( "u_Color" );
	mat4 P = m_Camera.GetCameraMat();

	// Draw every Drawable
	for ( Drawable& dr : m_vDrawables )
	{
		mat4 PMV = P * dr.GetMV();
		vec4 c = dr.GetColor();
		glUniformMatrix4fv( pmvHandle, 1, GL_FALSE, glm::value_ptr( PMV ) );
		glUniform4fv( clrHandle, 1, glm::value_ptr( c ) );
		dr.Draw();
	}

	//if ( m_bDrawContacts )
	{
		Drawable d1, d2;
		std::array<Drawable *, 2> pContactDr = { &d1, &d2 };
		for ( Contact& c : m_liSpeculativeContacts )
		{
			// NYI
			c.InitDrawable( pContactDr );
			for ( Drawable * pDr : pContactDr )
			{
				mat4 PMV = P * pDr->GetMV();
				vec4 c = pDr->GetColor();
				glUniformMatrix4fv( pmvHandle, 1, GL_FALSE, glm::value_ptr( PMV ) );
				glUniform4fv( clrHandle, 1, glm::value_ptr( c ) );
				pDr->Draw();
			}
		}
	}

	// Swap window
	SDL_GL_SwapWindow( m_pWindow );
}

void Scene::Update()
{
	// If we haven't paused the RB simulation)
	if ( m_bPauseCollision == false )
	{
		m_liSpeculativeContacts.clear();

		// Reset the contact list and find contacts
		int nCollisions( 0 );
		float fTotalEnergy( 0.f );

		// Integrate objects
		for ( RigidBody2D& rb : m_vRigidBodies )
			rb.Integrate( g_fTimeStep );

		// Get out if there's less than 2
		if ( m_vRigidBodies.size() < 2 )
			return;

		// For every plane
		for ( Plane& P : m_vCollisionPlanes )
		{
			// For every RB
			for ( auto itOuter = m_vRigidBodies.begin(); itOuter != m_vRigidBodies.end(); ++itOuter )
			{
				const float fGrav = 20.f;
				itOuter->v2Force += -vec2( 0, fGrav );

				// Check Plane RB collisions
				m_liSpeculativeContacts.push_back( RigidBody2D::GetSpeculativeContact( &P, &*itOuter ) );

				// Check every one against the other
				for ( auto itInner = itOuter + 1; itInner != m_vRigidBodies.end(); ++itInner )
				{
					// Skip if both have negative mass
					if ( itOuter->fMass < 0 && itInner->fMass < 0 )
						continue;

					m_liSpeculativeContacts.push_back( RigidBody2D::GetSpeculativeContact( &*itOuter, &*itInner ) );
				}

				// Increment total energy while we're at it
				fTotalEnergy += itOuter->GetKineticEnergy();
			}
		}

		// std::cout << fTotalEnergy << std::endl;

		// Solve contacts
		m_ContactSolver.Solve( m_liSpeculativeContacts );
	}
}


int Scene::AddDrawable( std::string strIqmFile, vec2 T, vec2 S, vec4 C, float theta /*= 0.f*/ )
{
	Drawable D;
	try
	{
		// Assume rotation about z for now
		fquat qRot( cos( theta / 2 ), sin( theta / 2 ) * vec3( 0, 0, 1 ) );
		D.Init( strIqmFile, C, quatvec( vec3( T, 0 ), qRot, quatvec::Type::TR ), S );
	}
	catch ( std::runtime_error )
	{
		return -1;
	}

	m_vDrawables.push_back( D );
	return (int) (m_vDrawables.size() - 1);
}

int Scene::AddRigidBody( RigidBody2D::EType eType, glm::vec2 v2Vel, glm::vec2 v2Pos, float fMass, float fElasticity, std::map<std::string, float> mapDetails )
{
	try
	{
		RigidBody2D rb;
		switch ( eType )
		{
			case RigidBody2D::EType::Circle:
			{
				float fRad = mapDetails.at( "r" );
				rb = Circle::Create( v2Vel, v2Pos, fMass, fElasticity, fRad );
				break;
			}
			case RigidBody2D::EType::AABB:
			{
				float w = mapDetails.at( "w" );
				float h = mapDetails.at( "h" );
				if ( eType == RigidBody2D::EType::AABB )
				{
					rb = AABB::Create( v2Vel, v2Pos, fMass, fElasticity, glm::vec2( w, h ) / 2.f );
				}
				break;
			}
			default:
				return -1;
		}

		m_vRigidBodies.push_back( rb );
		return m_vRigidBodies.size() - 1;
	}
	catch ( std::out_of_range )
	{
		std::cerr << "Error! Invalid details provided when creating Rigid Body!" << std::endl;
	}

	return -1;
}

int Scene::AddCollisionPlane( glm::vec2 N, float d )
{
	N = glm::normalize( N );
	m_vCollisionPlanes.push_back( { N, d } );
	
	// This function will add the drawable for now
	const float fLarge = 1000.f;
	vec2 S( fLarge );
	vec2 T = (d - fLarge / 2) * N;
	float fTheta = acos( glm::dot( N, vec2( 1, 0 ) ) );
	int ixDrawable = AddDrawable( "../models/quad.iqm", T, S, vec4( 0, 0, 0, 1 ), fTheta );

	return (int) (m_vCollisionPlanes.size() - 1);
}

const Shader * Scene::GetShaderPtr() const
{
	return  &m_Shader;
}

const Camera * Scene::GetCameraPtr() const
{
	return &m_Camera;
}

const Plane * Scene::GetPlane( const size_t planeIdx ) const
{
	if ( planeIdx < m_vCollisionPlanes.size() )
		return &m_vCollisionPlanes[planeIdx];

	throw std::runtime_error( "Error: Drawable index out of bound!" );
	return nullptr;
}

const Drawable * Scene::GetDrawable( const size_t drIdx ) const
{
	if ( drIdx < m_vDrawables.size() )
		return &m_vDrawables[drIdx];

	throw std::runtime_error( "Error: Drawable index out of bound!" );
	return nullptr;
}

const RigidBody2D * Scene::GetRigidBody2D( const size_t rbIdx ) const
{
	if ( rbIdx < m_vRigidBodies.size() )
		return &m_vRigidBodies[rbIdx];

	throw std::runtime_error( "Error: RB index out of bound!" );
	return nullptr;
}

std::list<Contact *> Scene::GetContacts() const
{
	std::list<Contact *> liRet;
	for ( const Contact& c : m_liSpeculativeContacts )
		liRet.push_back( (Contact *)&c );
	return liRet;
}

void Scene::SetQuitFlag( bool bQuit )
{
	m_bQuitFlag = bQuit;
}

bool Scene::GetQuitFlag() const
{
	return m_bQuitFlag;
}

void Scene::SetPauseCollision( bool bPauseCollision )
{
	m_bPauseCollision = bPauseCollision;
}

bool Scene::GetPauseCollision() const
{
	return m_bPauseCollision;
}

void Scene::SetDrawContacts( bool bDrawContacts )
{
	m_bDrawContacts = bDrawContacts;
}

bool Scene::GetDrawContacts() const
{
	return m_bDrawContacts;
}

bool Scene::InitDisplay( std::string strWindowName, vec4 v4ClearColor, std::map<std::string, int> mapDisplayAttrs )
{
	SDL_Window * pWindow = nullptr;
	SDL_GLContext glContext = nullptr;

	try
	{
		pWindow = SDL_CreateWindow( strWindowName.c_str(),
						  mapDisplayAttrs["posX"],
						  mapDisplayAttrs["posY"],
						  mapDisplayAttrs["width"],
						  mapDisplayAttrs["height"],
						  mapDisplayAttrs["flags"] );

		SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, mapDisplayAttrs["glMajor"] );
		SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, mapDisplayAttrs["glMinor"] );

		SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );
		SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, mapDisplayAttrs["doubleBuf"] );

		glContext = SDL_GL_CreateContext( pWindow );
		if ( glContext == nullptr )
		{
			std::cout << "Error creating opengl context" << std::endl;
			return false;
		}

		//Initialize GLEW
		glewExperimental = GL_TRUE;
		GLenum glewError = glewInit();
		if ( glewError != GLEW_OK )
		{
			printf( "Error initializing GLEW! %s\n", glewGetErrorString( glewError ) );
			return false;
		}

		SDL_GL_SetSwapInterval( mapDisplayAttrs["vsync"] );

		glClearColor( v4ClearColor.x, v4ClearColor.y, v4ClearColor.z, v4ClearColor.w );

		glEnable( GL_DEPTH_TEST );
		glDepthMask( GL_TRUE );
		glDepthFunc( GL_LESS );
		glEnable( GL_MULTISAMPLE_ARB );

	}
	catch ( std::out_of_range e )
	{
		if ( pWindow )
			SDL_DestroyWindow( pWindow );
		
		if ( glContext )
			SDL_GL_DeleteContext( glContext );

		return false;
	}

	m_pWindow = pWindow;
	m_GLContext = glContext;

	return true;
}