"""
Visualization of a simulation
"""
import visual
import numpy as np
import rigidmotion as rm

class World(object):
    """ A drawable version of rigidmotion.World
    """
    
    def __init__(self, world):
        self._world = world
        self._scene = visual.display()
        self.bodies = []

        for b in self._world.bodies:
            self.bodies.append(Body(b))

    def update(self):
        pass

class Body(object):
    """ A drawable version of rigidmotion.Body
    """
    
    def __init__(self, body):
        self._body = body
        self.frames = [draw_frame(pose=body.pose, label=body.frames[0].name)]
        for f in body.frames[1:]:
            #pose = np.dot(body.pose,f.pose)
            self.frames.append(draw_frame(pose=f.pose, label=f.name, parent=self.frames[0]))


def draw_frame(pose=np.eye(4), label=None, parent=None):
    """Draw the arrows and label of a frame.
    """
    (pos,axis,up) = htr_to_visual(pose)
    f = visual.frame(frame=parent, pos=pos, axis=axis, up=up)
    visual.arrow(frame=f, axis=(1,0,0), color=visual.color.red)
    visual.arrow(frame=f, axis=(0,1,0), color=visual.color.green)
    visual.arrow(frame=f, axis=(0,0,1), color=visual.color.blue)
    visual.label(frame=f, text=label)
    return f
    
def htr_to_visual(pose):
    """Convert an homogeneous matrix to visual pos,axis an up vectors.
    """
    pos = pose[0:2,3]
    axis = pose[0:2,0]
    up = pose[0:2,1]
    return (pos,axis,up)
