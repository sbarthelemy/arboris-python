# coding=utf-8
"""
Visualization of a simulation
"""
import numpy as np
import rigidmotion as rm
from enthought.mayavi import mlab


class World(object):
    """ A drawable version of rigidmotion.World
    """
    
    def __init__(self, world):
        self._world = world
        self.bodies = []

        for b in self._world.bodies:
            self.bodies.append(Body(b))

    def update(self):
        for b in self.bodies:
            b.update()
            







class Body(object):
    """ A drawable version of rigidmotion.Body
    """
    
    def __init__(self, body):
        self._body = body
        self.frames = [draw_frame(pose=body.pose, label=body.frames[0].name)]
        for f in body.frames[1:]:
            self.frames.append(draw_frame(pose=f.pose, label=f.name, parent=self.frames[0]))
        
    def update(self):
        for f in self.frames:
            if f.parent == None:
                pos = self.pose
            else:
                pos = np.dot(f.parent.pose, self.pose)
            origin = np.array([[pos(0,3), pos(1,3), pos(2,3)]])
            f.vx.mlab_source.points = origin
            f.vy.mlab_source.points = origin
            f.vz.mlab_source.points = origin
            f.vx.mlab_source.vectors([[pos(0,0), pos(1,0), pos(2,0)]])
            f.vy.mlab_source.vectors([[pos(0,1), pos(1,1), pos(2,1)]])
            f.vZ.mlab_source.vectors([[pos(0,2), pos(1,2), pos(2,2)]])
            
        
        
def draw_frame(pose=np.eye(4), label=None, parent=None):
    """Draw the arrows and label of a frame.
    """
    if parent == None:
        pos = pose
    else:
        pos = np.dot(parent.pose, pose)
        
    f.vx = mlab.quiver3d(pos(0,3), pos(1,3), pos(2,3), pos(0,0), pos(1,0), pos(2,0), color=(1,0,0))
    f.vy = mlab.quiver3d(pos(0,3), pos(1,3), pos(2,3), pos(0,1), pos(1,1), pos(2,1), color=(0,1,0))
    f.vz = mlab.quiver3d(pos(0,3), pos(1,3), pos(2,3), pos(0,2), pos(1,2), pos(2,2), color=(0,0,1))
    f.parent = parent
    f.label = label
    return f
