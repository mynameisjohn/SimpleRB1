#include "GL_Util.h"
#include "InitPython.h"
#include "Scene.h"
#include "CollisionFunctions.h"

#include <SDL.h>
#include <pyliaison.h>

int main(int argc, char ** argv)
{
	// Expose all python modules and initialize interpreter
	ExposeAll();
	pyl::initialize();

	// Get main python script object
	pyl::Object obMainScript = pyl::Object::from_script( "../scripts/main.py" );

	// Declare scene, initialize from python
	Scene S;
	obMainScript.call( "Initialize", &S );

	// Main loop
	bool bQuit = false;
	while ( bQuit == false )
	{
		// Handle events in python
		SDL_Event e{ 0 };
		while ( SDL_PollEvent( &e ) )
		{
			obMainScript.call( "HandleEvent", &e, &S );
		} 

		// Call the update function in python, maybe quit
		obMainScript.call( "Update", &S );
		bQuit = S.GetQuitFlag();
	}

	// Tear down interpreter and get out
	pyl::finalize();
	return 0;
}