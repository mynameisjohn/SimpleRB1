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

import random
import itertools

# Global containers
g_liEnts = []
g_liPlanes = []
g_InputManager = InputManager.InputManager(None, None, None)
g_SoftMouseManager = None

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
        Entity.nEntsCreated += 1


    def GetDrawableComponent(self):
        return pylDrawable.Drawable(self.cScene.GetDrawable(self.drIdx))
        
    def GetCollisionComponent(self):
        return pylRigidBody2D.RigidBody2D(self.cScene.GetRigidBody2D(self.rbIdx))

    def Update(self):
        # Get position of the collision component
        c = self.GetCollisionComponent()
        pos = c.Position()

        global g_InputManager
        if g_InputManager.mouseMgr.IsButtonPressed(sdl2.SDL_BUTTON_LEFT):
            # If mouse is down, create an attractive potential
            worldMousePos = g_InputManager.mouseMgr.fnMouseToWorld(g_InputManager.mouseMgr.mousePos)
            dist = [p - w for w, p in zip(worldMousePos, pos)]
            r2_inv = 1. / max(0.1, sum(di**2 for di in dist))
            F = [-80.*di * r2_inv for di in dist]
        else:
            # Otherwise just do gravity
            F = [0., -75. * c.GetMass()]

        # Apply force
        c.ApplyForce(F)

        # Update drawable transform
        self.GetDrawableComponent().SetPos2D(pos)

# Doesn't do a whole lot for now
class Plane:
    def __init__(self, cScene, N, d):
        self.cScene = cScene
        self.N = N
        self.d = d
        self.idx = cScene.AddCollisionPlane(N, d)

class SoftEntity:
    def __init__(self, cScene, ixSB, ixDR):
        self.cScene = cScene
        self.ixSB = ixSB
        self.ixDR = ixDR
        self.idLastOverlapped = None

    def GetSoftBody(self):
        return pylShape.Shape(self.cScene.GetSoftBody2D(self.ixSB))

    def GetDrawable(self):
        return pylDrawable.Drawable(self.cScene.GetDrawable(self.ixDR))

    def SetIsActive(self, bActive):
        self.GetSoftBody().SetIsActive(bActive)
        self.GetDrawable().SetIsActive(bActive)

class MouseSoftBodyManager:
    def __init__(self, cScene, liSoftEntities):
        if not(all(isinstance(lse, SoftEntity) for lse in liSoftEntities)):
            raise RuntimeError('poorly formed soft entity list')

        self.cScene = cScene
        self.liSoftEntities = liSoftEntities
        for e in self.liSoftEntities:
            e.SetIsActive(False)

        self.entGen = itertools.cycle(self.liSoftEntities)
        self.activeEnt = next(self.entGen)

    def Advance(self, bActivateNext):
        self.activeEnt.SetIsActive(False)
        self.activeEnt = next(self.entGen)
        if bActivateNext:
            self.activeEnt.SetIsActive(bActivateNext)

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
        rbPos = [3,6],
        rbVel = [0,0],
        rbMass = 1,
        rbElasticity = 1,
        rbDetails = {'w' : 1, 'h' : 1},
        drIQMFile = '../models/quad.iqm',
        drPos = [0,0],
        drScale = [1,1],
        drColor = [1,1,1,1]))

    g_liEnts.append(Entity(cScene,
        rbPrim = pylShape.Circle,
        rbPos = [-3,0],
        rbVel = [0,0],
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

    # Create soft body entities (that follow mouse))
    liSoftEntities = []
    # quad
    ixSoftQuadDR = cScene.AddDrawableIQM('../models/quad.iqm', # VAO key
                          [0,0],            # pos
                          [1,1],            # scale
                          [0,1,1,1],        # color
                          0 )               # rotation
    ixSoftQuadSB = cScene.AddSoftBody(pylShape.AABB, [0, 0], {'w' : 1, 'h' : 1})
    liSoftEntities.append(SoftEntity(cScene, ixSoftQuadSB, ixSoftQuadDR))
    # circle
    ixSoftCircDR = cScene.AddDrawableIQM('../models/circle.iqm', # VAO key
                          [0,0],            # pos
                          [1,1],            # scale
                          [0,1,1,1],        # color
                          0 )               # rotation
    ixSoftCircSB = cScene.AddSoftBody(pylShape.Circle, [0, 0], {'r' : .5})
    liSoftEntities.append( SoftEntity(cScene, ixSoftCircSB, ixSoftCircDR))
    # triangle
    triVerts = [[0,0,0],[-2,-1,0],[-2,1,0]]
    CentroidToOrigin(triVerts)
    ixSoftTriDR = cScene.AddDrawableTri('tri1', # VAO key
                          triVerts,         # verts
                          [0,0],            # pos
                          [1,1],            # scale
                          [0,1,1,1],        # color
                          0 )               # rotation
    ixSoftTriSB = cScene.AddSoftBody(pylShape.Triangle, [0, 0], { 'aX' : triVerts[0][0], 'aY' : triVerts[0][1],
                                                    'bX' : triVerts[1][0], 'bY' : triVerts[1][1],
                                                    'cX' : triVerts[2][0], 'cY' : triVerts[2][1]})   
    liSoftEntities.append(SoftEntity(cScene, ixSoftTriSB, ixSoftTriDR))
   
    global g_SoftMouseManager
    g_SoftMouseManager = MouseSoftBodyManager(cScene, liSoftEntities)

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

    def MouseToWorld(mousePos, sb = None, dr = None):
        nonlocal cCamera, cScene, worldDims
        fWidth = float(cCamera.GetScreenWidth())
        fHeight = float(cCamera.GetScreenHeight())
        nrmScreenPos = [p / f for p, f in zip(mousePos, [fWidth, fHeight])]
        nrmScreenPos[1] = 1. - nrmScreenPos[1]
        
        worldPos = [0., 0.]
        worldPos[0] = worldDims[0] + nrmScreenPos[0] * (worldDims[1]-worldDims[0])
        worldPos[1] = worldDims[2] + nrmScreenPos[1] * (worldDims[3]-worldDims[2])

        if sb is not None:
            sb.SetCenterPos(worldPos)
        if dr is not None:
            dr.SetPos2D(sb.Position())

        return worldPos

    def fnMotion(mouseMgr):
        global g_SoftMouseManager
        nonlocal cCamera, cScene, worldDims
        sb = g_SoftMouseManager.activeEnt.GetSoftBody()
        dr = g_SoftMouseManager.activeEnt.GetDrawable()
        if not(sb.GetIsActive() and dr.GetIsActive()):
            return

        MouseToWorld(mouseMgr.mousePos, sb, dr)

    class lbHandler:
        def __init__(self, bOn):
            self.bOn = bool(bOn)
        def __call__(self, btn, mouseMgr):
            global g_SoftMouseManager
            nonlocal cScene
            g_SoftMouseManager.activeEnt.SetIsActive(self.bOn)
            if self.bOn:
                sb = g_SoftMouseManager.activeEnt.GetSoftBody()
                dr = g_SoftMouseManager.activeEnt.GetDrawable()
                MouseToWorld(mouseMgr.mousePos, sb, dr)

    btnLeftMouse = InputManager.Button(sdl2.SDL_BUTTON_LEFT, fnDown = lbHandler(True), fnUp = lbHandler(False))

    def fnWheel(mouseMgr, sdlWheelEvent):
        global g_SoftMouseManager
        nonlocal cScene
        if sdlWheelEvent.y != 0:
            g_SoftMouseManager.Advance(True)
            sb = g_SoftMouseManager.activeEnt.GetSoftBody()
            dr = g_SoftMouseManager.activeEnt.GetDrawable()
            MouseToWorld(mouseMgr.mousePos, sb, dr)

    mouseMgr = InputManager.MouseManager([btnLeftMouse], fnMotion = fnMotion, fnWheel = fnWheel)
    mouseMgr.fnMouseToWorld = MouseToWorld

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

    for i in range(len(g_liEnts)):
        for sb in g_SoftMouseManager.liSoftEntities:
            p1 = sb.GetSoftBody().c_ptr
            p2 = g_liEnts[i].GetCollisionComponent().c_ptr
            if cScene.GetIsColliding(p1, p2):
                print('soft')
        for j in range(i+1, len(g_liEnts)):
            p1 = g_liEnts[i].GetCollisionComponent().c_ptr
            p2 = g_liEnts[j].GetCollisionComponent().c_ptr
            if cScene.GetIsColliding(p2, p1):
                print('hard')

# Handle SDL2 events
def HandleEvent(pSdlEvent, pScene):
    # Delegate events to the input manager
    global g_InputManager
    sdlEvent = ctype_from_addr(pSdlEvent, sdl2.events.SDL_Event)
    g_InputManager.HandleEvent(sdlEvent)
    