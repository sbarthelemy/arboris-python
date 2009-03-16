"""
Visualization of a simulation
"""
import visual as v
import rigidmotion as rm

class WorldVisualizer(object):
    
    def __init__(self, world):
        self._world = world

    def draw(self):
        self._world.geometric()
        self._scene = v.display()
        self._body_frames = []

        for b in self._world.bodies:
            DrawableBody(b)

        for j in self._world.joints:
            pass


class DrawableBody(object):
    def __init__(self,body):
        self.body = body
        (pos,axis,up) = htr_to_visual(body.pose)
        self.frame = v.frame(pos=pos, axis=axis, up=up)
        draw_frame(self.frame)
        v.label(frame=self.frame, text=self.body.name)
        
def draw_frame(frame=None):
    v.arrow(frame=frame, axis=(1,0,0),color=v.color.red)
    v.arrow(frame=frame, axis=(0,1,0),color=v.color.green)
    v.arrow(frame=frame, axis=(0,0,1),color=v.color.blue)
    
def htr_to_visual(pose):
    pos = pose[0:2,3]
    axis = pose[0:2,0]
    up = pose[0:2,1]
    return (pos,axis,up)
