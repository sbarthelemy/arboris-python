# coding=utf-8
"""
Visualization of a simulation
"""
import visual
import numpy as np
import rigidmotion as rm

scale=1

class World(object):
    """ A drawable version of rigidmotion.World
    """
    
    def __init__(self, world,scale=1):
        self._world = world
        self._scene = visual.display()
        self._scale = scale
        self.bodies = []
        self._scene.autoscale = False
        self.movecoeff = 0.25

        for b in self._world.bodies:
            self.bodies.append(Body(b))

    def update(self):
        for b in self.bodies:
            b.update()
        if self._scene.kb.keys:
            keypressed = self._scene.kb.getkey()
	    colindir = self._scene.forward - (visual.dot(self._scene.forward, self._scene.up) * self._scene.up)
	    colindir = visual.norm(colindir)
	    orthodir = visual.cross(colindir, self._scene.up)
	    if keypressed == 'left':
	        self._scene.center -= self.movecoeff*orthodir
	    if keypressed == 'right':
	        self._scene.center += self.movecoeff*orthodir
	    if keypressed == 'page down':
	        self._scene.center -= self.movecoeff*self._scene.up
	    if keypressed == 'page up':
	        self._scene.center += self.movecoeff*self._scene.up
	    if keypressed == 'down':
	        self._scene.center -= self.movecoeff*orthodir
	    if keypressed == 'up':
	        self._scene.center += self.movecoeff*orthodir

class Body(object):
    """ A drawable version of rigidmotion.Body
    """
    
    def __init__(self, body):
        self._body = body
        self.frames = [draw_frame(
            pose=body.pose,
            label=body.frames[0].name,
            length=scale)]
        for f in body.frames[1:]:
            self.frames.append(draw_frame(pose=f.pose, label=f.name,
                                          parent=self.frames[0],length=scale))
        
    def update(self):
        (pos,axis,up) = htr_to_visual(self._body.pose)
        self.frames[0].pos = pos
        self.frames[0].axis = axis
        self.frames[0].up = up
        
        
def draw_frame(pose=np.eye(4), label=None, parent=None, length=1):
    """Draw the arrows and label of a frame.
    """
    (pos,axis,up) = htr_to_visual(pose)
    f = visual.frame(frame=parent, pos=pos, axis=axis, up=up)
    visual.arrow(frame=f, axis=(1,0,0), length=length, color=visual.color.red)
    visual.arrow(frame=f, axis=(0,1,0), length=length, color=visual.color.green)
    visual.arrow(frame=f, axis=(0,0,1), length=length, color=visual.color.blue)
    visual.label(frame=f, text=label)
    return f
    
def htr_to_visual(pose):
    """Convert an homogeneous matrix to visual pos,axis an up vectors.
    """
    pos = pose[0:2,3]
    axis = pose[0:2,0]
    up = pose[0:2,1]
    return (pos,axis,up)
