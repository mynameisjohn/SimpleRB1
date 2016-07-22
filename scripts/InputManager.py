# I'd like to make this agnostic of
# SDL events, but for now this works
import sdl2.events

# Button class, own an identifier (code)
# and a callback that is called when the
# button is pressed and released
# TODO repeat?
class Button:
    def __init__(self, code, **kwargs):
        # Store code, set state to false
        self.code = code
        self.state = False

        # Try and get fnDown if they provided it
        self.fnDown = None
        if 'fnDown' in kwargs.keys():
            if hasattr(kwargs['fnDown'], '__call__'):
                self.fnDown = kwargs['fnDown']
            else:
                raise ValueError('Error: button down function not callable!')

        # same with fnUp
        self.fnUp = None
        if 'fnUp' in kwargs.keys():
            if hasattr(kwargs['fnUp'], '__call__'):
                self.fnUp = kwargs['fnUp']
            else:
                raise ValueError('Error: button up function not callable!')

        # Is this necessary?
        if self.fnDown is None and self.fnUp is None:
            raise ValueError('Error: no button functions provided!')

    # This isn't really helpful, but why not?
    def __hash__(self):
        return hash(self.code)

    # Gets called when the button is pressed
    def Toggle(self, mgr):
        # Update toggle state
        oldState = self.state
        self.state = not(self.state)
        # depending on new state, do fnUp or fnDown
        fnToggle = self.fnDown if self.state else self.fnUp
        if fnToggle is not None:
            fnToggle(self, mgr)

class KeyboardManager:
    def __init__(self, liKeys):
        if not(all(isinstance(k, Button) for k in liKeys)):
            raise TypeError('Error: All keys registered must be Buttons')

        # diKeys is initialized with the registered buttons
        # but will store bools for unregistered keys
        self.diKeys = {k.code : k for k in liKeys}

    # Handle an SDL2 key event
    def HandleKey(self, keyEvent):
        # Get the keycode
        keyCode = keyEvent.keysym.sym
        # I only care about non-repeated for now
        if keyEvent.repeat == False:
            # If we have this in our dict
            if keyCode in self.diKeys.keys():
                # If it's a registered button
                if isinstance(self.diKeys[keyCode], Button):
                    # Delegate to the button
                    self.diKeys[keyCode].Toggle(self)
                # Otherwise it should be a bool, so flip it
                else:
                    self.diKeys[keyCode] = not(self.diKeys[keyCode])
            # If it's new and it's a keydown event, add a bool to the dict
            elif keyEvent.type == sdl2.events.SDL_KEYDOWN:
                self.diKeys[keyCode] = True

    # If a button was registered, return it, otherwise return None
    def GetButton(self, keyCode):
        if keyCode in self.diKeyStates.keys():
            if isinstance(self.diKeys[keyCode], Button):
                return self.diKeyStates[keyCode]
        return None

    # If a button was registered return it's state
    # Otherwise if we have it return the bool, else False
    def IsKeyDown(self, keyCode):
        # If we have it in our button dict, return that
        if self.GetButton(keyCode) is not None:
            return self.GetButton(keyCode).state
        # If we have it as a bool, return that
        elif keyCode in self.diKeyStates.keys():
            return self.diKeyStates[keyCode]
        # If we've never seen it, I guess it's not down?
        return False

# Mouse manager, handles motion and lb/rb buttons
class MouseManager:
    def __init__(self, liButtons, motionCallback = None):
        # initial mouse pos
        self.mousePos = [0, 0]

        # Store buttons
        self.diButtons = {m.code : m for m in liButtons}

        # Better be a function if provided
        if hasattr(motionCallback, '__call__'):
            self.motionCallback = motionCallback
        else:
            self.motionCallback = None
     
    # Handle sdl2 mouse and motion events   
    def HandleMouse(self, sdlEvent):
        # buttons
        if (sdlEvent.type == sdl2.events.SDL_MOUSEBUTTONUP or
                    sdlEvent.type == sdl2.events.SDL_MOUSEBUTTONDOWN):
            btn = sdlEvent.button.button
            if btn in self.diButtons.keys():
                self.diButtons[btn].Toggle(self)

        # motion
        elif sdlEvent.type == sdl2.events.SDL_MOUSEMOTION:
            m = sdlEvent.motion
            self.mousePos = [m.x, m.y]
            if self.motionCallback is not None:
                self.motionCallback(self)

# Input manager, owns a mouse and keyboard handler
# as well as reference to C++ scene class (needed?)
class InputManager:
    def __init__(self, cScene, keyMgr, mouseMgr):
        self.cScene = cScene
        self.keyMgr = keyMgr
        self.mouseMgr = mouseMgr

    # Handle some sdl2 event
    def HandleEvent(self, sdlEvent):
        # Handle the quit event by setting the scene's quit flag
        if sdlEvent.type == sdl2.events.SDL_QUIT:
            self.cScene.SetQuitFlag(True)

        # Give key events to the keyboard manager
        if self.keyMgr is not None:
            if (sdlEvent.type == sdl2.events.SDL_KEYDOWN or
                    sdlEvent.type == sdl2.events.SDL_KEYUP):
                self.keyMgr.HandleKey(sdlEvent.key)

        # And mouse events to the mouse manager (will get motion and button)
        if self.mouseMgr is not None:
            if (sdlEvent.type == sdl2.events.SDL_MOUSEBUTTONUP or
                    sdlEvent.type == sdl2.events.SDL_MOUSEBUTTONDOWN or
                    sdlEvent.type == sdl2.events.SDL_MOUSEMOTION):
                self.mouseMgr.HandleMouse(sdlEvent)
