# coding=utf-8
"""
Visualization of a simulation
"""
import numpy as np
import visu
import visual

class World(visu.World):
    """ A drawable version of rigidmotion.World
    """
    #TODO: voir s'il n'y a pas de fonction overload de la fonction __init__
    def __init(self):
        self._scene = visual.display()
    
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
    
    def add_body(self, added_body):
        self.bodies.append(Body(added_body))
        
        
class Body(visu.Body):
    """ A drawable version of rigidmotion.Body
    """
    def draw_body(self):
        if self._body.pose != None:
            #self.frames[0] = self.draw_frame(self._body.pose, label=self._body.frames[0].name);
            self.frames.append(self.draw_frame(self._body.pose, label=self._body.frames[0].name));
            for f in self._body.frames:
                nf = self.draw_frame(f.pose, label=f.name, parent=self.frames[0])
                self.frames.append(nf);
                self.draw_link(self.frames[0], nf)
        
    def update(self):
        (pos,axis,up) = htr_to_visual(self._body.pose)
        self.frames[0].pos = pos
        self.frames[0].axis = axis
        self.frames[0].up = up
        visual.rate(10)
        
    def draw_frame(self, pose=np.eye(4), label=None, parent=None, length=1):
        """Draw the arrows and label of a frame.
        """
        (pos,axis,up) = self.htr_to_visual(pose)
        f = visual.frame(frame=parent, pos=pos, axis=axis, up=up)
        visual.arrow(frame=f, axis=(1,0,0), length=length, color=visual.color.red)
        visual.arrow(frame=f, axis=(0,1,0), length=length, color=visual.color.green)
        visual.arrow(frame=f, axis=(0,0,1), length=length, color=visual.color.blue)
        #visual.label(frame=f, text=label)
        return f
    
    def draw_link(self, start_frame, end_frame):
        pass
    
    def htr_to_visual(self, pose):
        """Convert an homogeneous matrix to visual pos,axis an up vectors.
        """
        pos = pose[0:2,3]
        axis = pose[0:2,0]
        up = pose[0:2,1]
        return (pos,axis,up)
