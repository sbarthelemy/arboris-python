
"""
This example shows that arboris creates energy. We do not know why

"""


import arboris.controllers
from arboris.observers import EnergyMonitor, PerfMonitor#, MassMonitor
from arboris.visu_osg import Drawer
from arboris.core import World, ObservableWorld, Body, SubFrame
from arboris.massmatrix import transport, cylinder
from arboris.homogeneousmatrix import transl
from arboris.joints import *

def add_robot(w):
    assert isinstance(w, World)
    
    l   = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
    m   = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]

    frame = w.ground
    for (length, mass) in zip(l, m):
        radius = length/10.
        M = transport(cylinder(length, radius, mass), 
                      transl(0., -length/2., 0.))
        body = Body(mass= M)
        joint = RzJoint()
        w.add_link(frame, joint, body)
        frame = SubFrame(body, transl(0., length, 0.))
    w.register(frame)
    w.init()
        
    # initial configuration
    w.getjoints()[0].gpos = [3.14159/4.]

    
from arboris.core import WorldObserver
from numpy import linalg, where

class MassMonitor(WorldObserver):
    def __init__(self, world):
        self._world = world
        self.mass_cond = []
        self.imp_cond = []
        self.c_rank=[]
        
    def init(self):
        pass
    
    def update(self, dt):
        self.mass_cond.append(linalg.cond(self._world._mass))
        self.imp_cond.append(linalg.cond(self._world._impedance))
        curr_rk = []
        for b in self._world.iterbodies():
            curr_rk.append( self.rank(b.jacobian) )
        self.c_rank.append(curr_rk)
    
    def rank(self, A, tol=1e-8):
        s = linalg.svd(A,compute_uv=0)
        return sum( where( s>tol, 1, 0 ) )

    def plot(self):
        from pylab import plot, show, xlabel, ylabel, title, figure

        plot(self.mass_cond)
        title('Condition of mass matrix during simulation')
        xlabel('step')
        ylabel('mass condition')

        figure()
        plot(self.imp_cond)
        title('Condition of impedance matrix during simulation')
        xlabel('step')
        ylabel('impedance condition')

        figure()
        plot(self.c_rank)
        title('jacobians rank')
        xlabel('step')
        ylabel('rank')
        
        show()

##### build section #####
w = ObservableWorld()       
add_robot(w)

##### controller section #####
weightc = arboris.controllers.WeightController(w)       
w.register(weightc)

##### observers section #####
nrj = EnergyMonitor(w)      
w.observers.append(nrj)

#perf = PerfMonitor(w)
#w.observers.append(perf)

mM = MassMonitor(w)
w.observers.append(mM)

##### dynamic section #####
w.init()        

##### simulation section #####
t= 0.; dt = 0.005
d = Drawer(w)
d.init()
#while not d.done():
while t < 3.06:
    t += dt
    w.update_dynamic()
    w.update_controllers(dt)
    w.integrate(dt)
    d.update(dt)

w.finish()

nrj.plot()
mM.plot()
    

