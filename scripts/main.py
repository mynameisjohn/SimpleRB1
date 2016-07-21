# Used for debugging
#import ptvsd
#ptvsd.enable_attach(secret = None)
#ptvsd.wait_for_attach()

# sdl2 stuff
import sdl2
import sdl2.audio
import sdl2.video
import sdl2.events

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

# Custom pyl modules
import pylScene
import pylShader
import pylCamera
import pylDrawable
import pylRigidBody2D

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
        drIdx = cScene.AddDrawable(kwargs['drIQMFile'],
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

    # create entities
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

# Update scene and draw
def Update(pScene):
    # construct pyl scene
    cScene = pylScene.Scene(pScene)
    for e in g_liEnts:
        e.Update()
    cScene.Update()
    cScene.Draw()

# Handle SDL2 events
def HandleEvent(pSdlEvent, pScene):
    # construct pyl scene
    cScene = pylScene.Scene(pScene)
    # construct sdl2 event
    sdlEvent = ctype_from_addr(pSdlEvent, sdl2.events.SDL_Event)
    # not a lot going on here for now, just exit handling
    if sdlEvent.type == sdl2.events.SDL_KEYUP and sdlEvent.key.keysym.sym == sdl2.keycode.SDLK_ESCAPE:
        cScene.SetQuitFlag(True)
    if sdlEvent.type == sdl2.events.SDL_QUIT:
        cScene.SetQuitFlag(True)
    