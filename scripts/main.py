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
import pylShape
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
        self.GetDrawableComponent().SetPos2D(self.GetCollisionComponent().Position())

# Doesn't do a whole lot for now
class Plane:
    def __init__(self, cScene, N, d):
        self.cScene = cScene
        self.N = N
        self.d = d
        self.idx = cScene.AddCollisionPlane(N, d)

def CentroidToOrigin(verts):
    N = len(verts)
    D = len(verts[0])
    if not(all(len(verts[i]) == D for i in range(N))):
        raise RuntimeError('poorly formed vertex list')
    centroid = [0. for j in range(D)]
    divFactor = 1./N
    for i in range(N):
        for j in range(D):
            centroid[j] += divFactor * verts[i][j]
    for i in range(N):
        for j in range(D):
            verts[i][j] -= centroid[j]

# Global containers
g_liEnts = []
g_liPlanes = []
g_InputManager = InputManager.InputManager(None, None, None)

# Initialize the scene
def Initialize(pScene):
    # construct pyl scene
    cScene = pylScene.Scene(pScene)

    screenDims = (800, 800)
    worldDims = (-10., 10, -10., 10)

    # init scene display
    if cScene .InitDisplay('SimpleRB1', [.1,.1,.1,1.],{
        'posX' : sdl2.video.SDL_WINDOWPOS_UNDEFINED,
        'posY' : sdl2.video.SDL_WINDOWPOS_UNDEFINED,
        'width' : screenDims[0],
        'height' : screenDims[1],
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
    cCamera.InitOrtho(*(screenDims+worldDims))

    # Set up static drawable handle
    pylDrawable.SetPosHandle(cShader.GetHandle('a_Pos'))

    # create entities, just two random ones for now
    g_liEnts.append(Entity(cScene,
        rbPrim = pylShape.AABB,
        rbPos = [6,6],
        rbVel = [-5,0],
        rbMass = 1,
        rbElasticity = 1,
        rbDetails = {'w' : 1, 'h' : 1},
        drIQMFile = '../models/quad.iqm',
        drPos = [0,0],
        drScale = [1,1],
        drColor = [1,1,1,1]))

    #g_liEnts.append(Entity(cScene,
    #    rbPrim = pylShape.Circle,
    #    rbPos = [-3,0],
    #    rbVel = [-5,0],
    #    rbMass = 1,
    #    rbElasticity = 1,
    #    rbDetails = {'r' : .5},
    #    drIQMFile = '../models/circle.iqm',
    #    drPos = [0,0],
    #    drScale = [1,1],
    #    drColor = [1,1,1,1]))

    # Create walls (planes)
    walls = [[1., 0.], [-1., 0.], [0., 1.], [0., -1.]]
    d = -8.
    for N in walls:
        g_liPlanes.append(Plane(cScene, N, d))

    # Testing triangle drawables
    # Vertices, translated such that centroid is at 0
    triVerts = [[0,0,0],[-2,1,0],[-2,-1,0]]
    CentroidToOrigin(triVerts)

    # add drawable and soft body
    ixTriDR = cScene.AddDrawableTri('tri1',      # VAO key
                          triVerts,    # verts
                          [0,0],       # pos
                          [1,1],       # scale
                          [0,0,1,1],   # color
                          0 )          # rotation

    # Kind of a pain
    ixTriSB = cScene.AddSoftBody(pylShape.Triangle, [0, 0], { 'aX' : triVerts[0][0], 'aY' : triVerts[0][1],
                                                    'bX' : triVerts[1][0], 'bY' : triVerts[1][1],
                                                    'cX' : triVerts[2][0], 'cY' : triVerts[2][1]})

    # Input handling
    # Quit function, triggered by escape
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

    def fnMotion(mouseMgr):
        nonlocal cScene, cCamera, worldDims, ixTriSB, ixTriDR
        fWidth = float(cCamera.GetScreenWidth())
        fHeight = float(cCamera.GetScreenHeight())
        nrmScreenPos = [p / f for p, f in zip(mouseMgr.mousePos, [fWidth, fHeight])]
        nrmScreenPos[1] = 1. - nrmScreenPos[1]
        
        worldPos = [0., 0.]
        worldPos[0] = worldDims[0] + nrmScreenPos[0] * (worldDims[1]-worldDims[0])
        worldPos[1] = worldDims[2] + nrmScreenPos[1] * (worldDims[3]-worldDims[2])

        sb = pylShape.Shape(cScene.GetSoftBody2D(ixTriSB))
        dr = pylDrawable.Drawable(cScene.GetDrawable(ixTriDR))
        sb.SetCenterPos(worldPos)
        dr.SetPos2D(sb.Position())
    mouseMgr = InputManager.MouseManager([], fnMotion = fnMotion)

    # Create input manager (no mouse for now)
    global g_InputManager
    keyManager = InputManager.KeyboardManager([btnQuitEsc, btnShowHideContacts, btnPlayPauseCollision])
    g_InputManager = InputManager.InputManager(cScene, keyManager, mouseMgr)

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
    