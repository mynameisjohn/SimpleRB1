# Used for debugging
#import ptvsd
#ptvsd.enable_attach(secret = None)
#ptvsd.wait_for_attach()

# sdl2 stuff
import sdl2
import sdl2.audio
import sdl2.video
import sdl2.events

# Custom pyl modules
import pylScene
import pylShader
import pylCamera
import pylDrawable
import pylRigidBody2D

# My input manager class
import InputManager

# Used to construct ctypes sdl2 object
# from pointer to object in C++
import ctypes
def ctype_from_addr(capsule, type):
    ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.c_void_p
    ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    addr = ctypes.pythonapi.PyCapsule_GetPointer(capsule, None)
    if (addr != 0):
        return type.from_address(addr)
    raise RuntimeError('Error constructing ctype object, invalid capsule address')


# Basic entity class, kind of gross at the moment
# but all I need it to do is facilitate RB-Drawable talk
class Entity:
    nEntsCreated = 0
    def __init__(self, cScene, **kwargs):
        # Create RB, get index
        rbIdx = cScene.AddRigidBody(kwargs['rbPrim'], 
                                    kwargs['rbVel'],
                                    kwargs['rbPos'],
                                    kwargs['rbMass'],
                                    kwargs['rbElasticity'],
                                    kwargs['rbDetails'])
        if rbIdx < 0:
            raise RuntimeError('Error creating rigid body')

        # Creat drawable, get index
        drIdx = cScene.AddDrawableIQM(kwargs['drIQMFile'],
                                    kwargs['drPos'],
                                    kwargs['drScale'],
                                    kwargs['drColor'],
                                    0.)
        if drIdx < 0:
            raise RuntimeError('Error creating rigid body')

        # If that went ok, store scene and indices
        self.cScene = cScene
        self.rbIdx = rbIdx
        self.drIdx = drIdx

        # Set entity IDs
        self.nID = Entity.nEntsCreated
        self.GetDrawableComponent().SetID(Entity.nEntsCreated)
        self.GetCollisionComponent().SetID(Entity.nEntsCreated)

    def GetDrawableComponent(self):
        return pylDrawable.Drawable(self.cScene.GetDrawable(self.drIdx))
        
    def GetCollisionComponent(self):
        return pylRigidBody2D.RigidBody2D(self.cScene.GetRigidBody2D(self.rbIdx))

    def Update(self):
        # Update drawable transform
        self.GetCollisionComponent().UpdateDrawable(self.GetDrawableComponent().c_ptr)

# Doesn't do a whole lot for now
class Plane:
    def __init__(self, cScene, N, d):
        self.cScene = cScene
        self.N = N
        self.d = d
        self.idx = cScene.AddCollisionPlane(N, d)

# Global containers
g_liEnts = []
g_liPlanes = []
g_InputManager = InputManager.InputManager(None, None, None)

# Initialize the scene
def Initialize(pScene):
    # construct pyl scene
    cScene = pylScene.Scene(pScene)

    # init scene display
    if cScene .InitDisplay('SimpleRB1', [.1,.1,.1,1.],{
        'posX' : sdl2.video.SDL_WINDOWPOS_UNDEFINED,
        'posY' : sdl2.video.SDL_WINDOWPOS_UNDEFINED,
        'width' : 800,
        'height' : 800,
        'flags' : sdl2.video.SDL_WINDOW_OPENGL | sdl2.video.SDL_WINDOW_SHOWN,
        'glMajor' : 3,
        'glMinor' : 0,
        'doubleBuf' : 1,
        'vsync' : 1
        }) == False:
        raise RuntimeError('Error initializing Scene Display')

    # Set up shader
    cShader = pylShader.Shader(cScene.GetShaderPtr())
    if cShader.Init('../shaders/simple.vert', '../shaders/simple.frag', True) == False:
        raise RuntimeError('Error initializing Shader')

    # Set up camera
    pylCamera.SetCamMatHandle(cShader.GetHandle('u_PMV'))
    cCamera = pylCamera.Camera(cScene.GetCameraPtr())
    cCamera.InitOrtho(-10., 10, -10., 10)

    # Set up static drawable handle
    pylDrawable.SetPosHandle(cShader.GetHandle('a_Pos'))

    # create entities, just two random ones for now
    g_liEnts.append(Entity(cScene,
        rbPrim = pylRigidBody2D.AABB,
        rbPos = [6,6],
        rbVel = [-5,0],
        rbMass = 1,
        rbElasticity = 1,
        rbDetails = {'w' : 1, 'h' : 1},
        drIQMFile = '../models/quad.iqm',
        drPos = [0,0],
        drScale = [1,1],
        drColor = [1,1,1,1]))

    g_liEnts.append(Entity(cScene,
        rbPrim = pylRigidBody2D.Circle,
        rbPos = [3,0],
        rbVel = [-5,0],
        rbMass = 1,
        rbElasticity = 1,
        rbDetails = {'r' : .5},
        drIQMFile = '../models/circle.iqm',
        drPos = [0,0],
        drScale = [1,1],
        drColor = [1,1,1,1]))

    # Create walls (planes)
    walls = [[1., 0.], [-1., 0.], [0., 1.], [0., -1.]]
    d = -8.
    for N in walls:
        g_liPlanes.append(Plane(cScene, N, d))

    # Testing triangle drawables
    #cScene.AddDrawableTri('tri1',                       # VAO key
    #                      [[0,0,0],[-2,1,0],[-2,-1,0]], # verts
    #                      [0,0],                        # pos
    #                      [1,1],                        # sale
    #                      [0,0,1,1],                    # color
    #                      0 )                           # rotation

    # Input handling

    # Quit function, triggered bye scape
    def fnQuitScene(btn, keyMgr):
        nonlocal cScene
        cScene.SetQuitFlag(True)
    btnQuitEsc = InputManager.Button(sdl2.keycode.SDLK_ESCAPE, fnUp = fnQuitScene)

    # Toggle contact drawing
    def fnShowHideContacts(btn, keyMgr):
        nonlocal cScene
        cScene.SetDrawContacts(not(cScene.GetDrawContacts()))
    btnShowHideContacts = InputManager.Button(sdl2.keycode.SDLK_c, fnUp = fnShowHideContacts)

    # Toggle collision detection and RB integration
    def fnPlayPauseCollision(btn, keyMgr):
        nonlocal cScene
        cScene.SetPauseCollision(not(cScene.GetPauseCollision()))
    btnPlayPauseCollision = InputManager.Button(sdl2.keycode.SDLK_SPACE, fnUp = fnPlayPauseCollision)

    # Create input manager (no mouse for now)
    global g_InputManager
    keyManager = InputManager.KeyboardManager([btnQuitEsc, btnShowHideContacts, btnPlayPauseCollision])
    g_InputManager = InputManager.InputManager(cScene, keyManager, None)

# Update scene and draw
def Update(pScene):
    # construct pyl scene
    cScene = pylScene.Scene(pScene)

    # Update entities, which updates drawable
    global g_liEnts
    for e in g_liEnts:
        e.Update()

    # Update and draw scene
    cScene.Update()
    cScene.Draw()

# Handle SDL2 events
def HandleEvent(pSdlEvent, pScene):
    # Delegate events to the input manager
    global g_InputManager
    sdlEvent = ctype_from_addr(pSdlEvent, sdl2.events.SDL_Event)
    g_InputManager.HandleEvent(sdlEvent)
    