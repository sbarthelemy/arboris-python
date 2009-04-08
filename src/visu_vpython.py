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
    def __init__(self, world):
        self._world = world
        self.bodies = []
        for b in self._world.bodies:
            self.add_body(b)
        self._scene = visual.display()
        self._scene.autoscale = 1
        self._scene.autocenter = 1
        self.update_frequency = 10

    def add_body(self, added_body):
        self.bodies.append(Body(added_body))
        
    def update(self):
        visual.rate(self.update_frequency)
        for b in self.bodies:
            b.update()
        if 0: ############ probleme: affiche une deuxieme fenetre ############
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
        

def draw_frame(pose=None, text=None, parent=None, length=1):
    """Draw the arrows and label of a frame.
    """
    if pose==None:
        pose = np.eye(4)
    (pos,axis,up) = htr_to_visual(pose)
    f = visual.frame(frame=parent, pos=pos, axis=axis, up=up)
    visual.arrow(frame=f, axis=(1,0,0), length=length, color=visual.color.red)
    visual.arrow(frame=f, axis=(0,1,0), length=length, color=visual.color.green)
    visual.arrow(frame=f, axis=(0,0,1), length=length, color=visual.color.blue)
    visual.label(frame=f, yoffset=-10, box=0, line=0, text=str(text))
    return f
    

def draw_link(ref_frame, start, end, color=(1,1,1)):
    link = visual.cylinder(
        frame = ref_frame, 
        pos = start, 
        axis = (end-start), 
        radius = 0.05, 
        color=color)
    return link


def htr_to_visual(pose):
    """Convert an homogeneous matrix to the ``pos``, ``axis`` an ``up`` vectors used by vpython
    """
    pos = pose[0:3,3]
    axis = pose[0:3,0]
    up = pose[0:3,1]
    return (pos,axis,up)


class Body(visu.Body):
    """ A drawable version of rigidmotion.Body
    """
    def draw_body(self):
        self.frames.append(draw_frame(
            self._body.pose, 
            self._body.frames[0].name))
        for f in self._body.frames[1:]:
            nf = draw_frame(f.pose, f.name, parent=self.frames[0])
            self.frames.append(nf)
            self.links.append(draw_link(self.frames[0], (0,0,0), nf.pos))

                
    def update(self):
        (pos,axis,up) = htr_to_visual(self._body.pose)
        self.frames[0].pos = pos
        self.frames[0].axis = axis
        self.frames[0].up = up
        

if __name__=='__main__':
    # testing!
    from worldfactory import triplehinge
    w = triplehinge()
    w.geometric()

    import visu_vpython as vpy
    vw = vpy.World(w)
    for t in range(100):
        w.joints[0].gpos=[t/20.]
        w.joints[1].gpos=[t/20.]
        w.geometric()
        vw.update()

    
