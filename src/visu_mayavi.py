# coding=utf-8
"""
Visualization of a simulation
"""
import numpy as np
import visu
from enthought.mayavi import mlab


class World(visu.World):
    """ A drawable version of rigidmotion.World
    """
    
    def add_body(self, added_body):
        self.bodies.append(Body(added_body))

class Body(visu.Body):
    """ A drawable version of rigidmotion.Body
    """
    def draw_body(self):
        return
        
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
            f.vx.mlab_source.vectors = np.array([[pos(0,0), pos(1,0), pos(2,0)]])
            f.vy.mlab_source.vectors = np.array([[pos(0,1), pos(1,1), pos(2,1)]])
            f.vZ.mlab_source.vectors = np.array([[pos(0,2), pos(1,2), pos(2,2)]])
            
        
    def draw_frame(self, pose=np.eye(4), label=None, parent=None):
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
    
    def draw_link(self):
        pass
        
    def draw_shape():
        pass
    
    
