'''
A box falling under gravity. Its trajectory is compared against the 
theoric one.
'''
from arboris.controllers import WeightController 
from arboris.core import World, simulate
from pylab import plot, show, legend, xlabel, ylabel, title
from arboris.core import Observer
from numpy import arange, dot, eye, array
import arboris.homogeneousmatrix as homogeneousmatrix
from arboris.core import Body, SubFrame
from arboris.joints import FreeJoint
from arboris.shapes import Box
import arboris.massmatrix as massmatrix    



class TrajLog(Observer):
    
    def __init__(self, frame, world):
        self.height = []
        self.timeline = []
        self.frame = frame #origin of frame should be the com
        self.world = world
        
    def init(self, world, timeline):
        pass
    
    def update(self, dt):
        self.timeline.append(self.world.current_time)
        self.height.append(dot(self.world.up, self.frame.pose[0:3,3]))
        print self.frame.pose[0:3,3]
        
    def finish(self):
        pass
    
    def get_theoric(self):
        mass = self.frame.body.mass[3,3]
        return [self.height[0]-9.81/2*mass*t**2 for t in self.timeline]
        
    def get_error(self):
        error = []
        for (sh, th) in zip(self.height, self.get_theoric()):
            error.append(sh-th)
        return error
         
    def plot_error(self):
        plot(self.timeline, self.get_error())
        
    def plot_height(self):
        plot(self.timeline, self.height,
             self.timeline, self.get_theoric())
        legend(('simulated', 'theoric'))

w = World()

if True:
    from arboris.homogeneousmatrix import transl
    H_bc = transl(1,1,1)
else:
    H_bc = eye(4)
half_extents = (.5,.5,.5)
mass = 1.
body = Body(
        name='box_body',
        mass=massmatrix.transport(massmatrix.box(half_extents, mass), H_bc))
subframe = SubFrame(body, H_bc, name="box_com")

if True:
    twist_c = array([0.,0.,0.,0.,0.,0.])
else:
    twist_c = array([1,1,1,0,0,0.])
twist_b = dot(homogeneousmatrix.adjoint(H_bc), twist_c)
freejoint = FreeJoint(gpos=homogeneousmatrix.inv(H_bc), gvel=twist_b)
w.add_link(w.ground, freejoint, body)
w.register(Box(subframe, half_extents))


weightc = WeightController()       
w.register(weightc)
obs = TrajLog(w.getframes()['box_com'], w)

from arboris.visu_osg import Drawer

timeline = arange(0,1,5e-3)
simulate(w, timeline, [obs, Drawer(w)])
    
time = timeline[:-1]
obs.plot_error()
show()
        
