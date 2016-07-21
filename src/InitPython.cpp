#include "Scene.h"

#include <pyliaison.h>
#include <iterator>
#include <string>
#include <map>

using namespace pyl;

bool ExposeEntComponent()
{
	const std::string strModuleName = "pylEntComponent";

	ModuleDef * pModDef = ModuleDef::Create<struct st_ecMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	pModDef->RegisterClass<EntComponent>( "EntComponent" );

	AddMemFnToMod( pModDef, EntComponent, SetID, void, int );
	AddMemFnToMod( pModDef, EntComponent, GetID, int );

	return true;
}

bool ExposeScene()
{
	ModuleDef * pModDef = CreateMod( pylScene );
	if ( pModDef == nullptr )
		return false;

	AddClassToMod( pModDef, Scene );

	AddMemFnToMod( pModDef, Scene, InitDisplay, bool, std::string, vec4, std::map<std::string, int> );
	AddMemFnToMod( pModDef, Scene, GetShaderPtr, const Shader * );
	AddMemFnToMod( pModDef, Scene, GetCameraPtr, const Camera * );
	AddMemFnToMod( pModDef, Scene, GetPlane, const Plane *, const size_t );
	AddMemFnToMod( pModDef, Scene, GetDrawable, const Drawable *, const size_t );
	AddMemFnToMod( pModDef, Scene, GetRigidBody2D, const RigidBody2D *, const size_t );
	AddMemFnToMod( pModDef, Scene, AddCollisionPlane, int, vec2, float );
	AddMemFnToMod( pModDef, Scene, AddDrawable, int, std::string, vec2, vec2, vec4, float );
	AddMemFnToMod( pModDef, Scene, AddRigidBody, int, RigidBody2D::EType, vec2, vec2, float, float, std::map<std::string, float> );
	AddMemFnToMod( pModDef, Scene, GetContacts, std::list<Contact *> );
	AddMemFnToMod( pModDef, Scene, GetQuitFlag, bool );
	AddMemFnToMod( pModDef, Scene, SetQuitFlag, void, bool );
	AddMemFnToMod( pModDef, Scene, GetDrawContacts, bool );
	AddMemFnToMod( pModDef, Scene, SetDrawContacts, void, bool );
	AddMemFnToMod( pModDef, Scene, GetPauseCollision, bool );
	AddMemFnToMod( pModDef, Scene, SetPauseCollision, void, bool );

	AddMemFnToMod( pModDef, Scene, Update, void );
	AddMemFnToMod( pModDef, Scene, Draw, void );

	return true;
}

bool ExposeShader()
{
	ModuleDef * pModDef = CreateMod( pylShader );
	if ( pModDef == nullptr )
		return false;

	AddClassToMod( pModDef, Shader );
	AddMemFnToMod( pModDef, Shader, Init, bool, std::string, std::string, bool );
	AddMemFnToMod( pModDef, Shader, PrintLog_V, int );
	AddMemFnToMod( pModDef, Shader, PrintLog_F, int );
	AddMemFnToMod( pModDef, Shader, PrintSrc_V, int );
	AddMemFnToMod( pModDef, Shader, PrintSrc_F, int );
	AddMemFnToMod( pModDef, Shader, PrintLog_P, int );
	AddMemFnToMod( pModDef, Shader, GetHandle, GLint, const std::string );

	return true;
}

bool ExposeCamera()
{
	ModuleDef * pModDef = CreateMod( pylCamera );
	if ( pModDef == nullptr )
		return false;

	AddClassToMod( pModDef, Camera );
	AddMemFnToMod( pModDef, Camera, InitOrtho, void, float, float, float, float );
	AddMemFnToMod( pModDef, Camera, InitPersp, void, float, float, float, float );

	pModDef->RegisterFunction<struct st_fnCamSetProjH>( "SetCamMatHandle", make_function( Camera::SetCamMatHandle ) );

	return true;
}

bool ExposeDrawable()
{
	ModuleDef * pModDef = CreateMod( pylDrawable );
	if ( pModDef == nullptr )
		return false;

	ModuleDef * pEntMod = ModuleDef::GetModuleDef( "pylEntComponent" );
	AddSubClassToMod( pModDef, Drawable, pEntMod, EntComponent );

	AddMemFnToMod( pModDef, Drawable, SetPos2D, void, vec2 );
	AddMemFnToMod( pModDef, Drawable, SetTransform, void, quatvec );
	AddMemFnToMod( pModDef, Drawable, SetColor, void, glm::vec4 );

	// The macro doesn't work out for static functions...
	auto SetPosHandle = Drawable::SetPosHandle;
	AddFnToMod( pModDef, SetPosHandle );

	auto SetColorHandle = Drawable::SetColorHandle;
	AddFnToMod( pModDef, SetColorHandle );

	return true;
}

bool ExposeRigidBody2D()
{
	ModuleDef * pModDef = CreateMod( pylRigidBody2D );
	if ( pModDef == nullptr )
		return false;

	ModuleDef * pEntMod = ModuleDef::GetModuleDef( "pylEntComponent" );
	AddSubClassToMod( pModDef, RigidBody2D, pEntMod, EntComponent );

	AddMemFnToMod( pModDef, RigidBody2D, UpdateDrawable, void, Drawable * );

	pModDef->SetCustomModuleInit( [] ( pyl::Object obModule )
	{
		obModule.set_attr( "Circle", RigidBody2D::EType::Circle );
		obModule.set_attr( "AABB", RigidBody2D::EType::AABB );
	} );

	return true;
}

bool ExposePlane()
{
	ModuleDef * pModDef = CreateMod( pylPlane );
	if ( pModDef == nullptr )
		return false;

	AddClassToMod( pModDef, Plane );
	AddMemFnToMod( pModDef, Plane, SetNormal, void, vec2 );
	AddMemFnToMod( pModDef, Plane, SetDist, void, float );

	return true;
}

bool ExposeContact()
{
	ModuleDef * pModDef = CreateMod( pylContact );
	if ( pModDef == nullptr )
		return false;

	AddClassToMod( pModDef, Contact );
	AddMemFnToMod( pModDef, Contact, GetBodyA, const RigidBody2D * );
	AddMemFnToMod( pModDef, Contact, GetBodyB, const RigidBody2D * );
	AddMemFnToMod( pModDef, Contact, IsColliding, bool );

	return true;
}

bool ExposeAll()
{
	// I just wanted to try a polymorphic lambda
	auto liExpose =
	{ ExposeEntComponent,
		ExposeScene,
		ExposeShader,
		ExposeCamera,
		ExposeDrawable,
		ExposeRigidBody2D,
		ExposeContact
	};
	return std::all_of( liExpose.begin(), liExpose.end(), [] ( auto fn ) { return fn(); } );
}

namespace pyl
{
	bool convert( PyObject * o, glm::vec2& v )
	{
		return convert_buf( o, &v[0], 2 );
	}
	bool convert( PyObject * o, glm::vec3& v )
	{
		return convert_buf( o, &v[0], 3 );
	}
	bool convert( PyObject * o, glm::vec4& v )
	{
		return convert_buf( o, &v[0], 4 );
	}
	bool convert( PyObject * o, glm::fquat& v )
	{
		return convert_buf( o, &v[0], 4 );
	}

	// Type? Should be part of this...
	bool convert( PyObject * o, quatvec& qv )
	{
		return convert_buf( o, &qv.vec[0], 7 );
	}

	template<typename eType>
	bool convertEnum( PyObject * o, eType& e )
	{
		int tmp( 0 );
		if ( bool bRet = convert( o, tmp ) )
		{
			e = (eType) tmp;
			return bRet;
		}
		return false;
	}

	bool convert( PyObject * o, RigidBody2D::EType& e )
	{
		return convertEnum<RigidBody2D::EType>( o, e );
	}

	PyObject * alloc_pyobject( const RigidBody2D::EType e )
	{
		return PyLong_FromLong( (long) e );
	}
}