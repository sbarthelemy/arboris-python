
"""
This example shows that arboris creates energy.

We can see it with four simulations, by toggling:

- initial velocity,
- gravity,
- the free-floating or fixed base.

"""


import arboris.controllers
from arboris.observers import EnergyMonitor, PerfMonitor#, MassMonitor
from arboris.visu_osg import Drawer
from arboris.core import World, ObservableWorld, Body, SubFrame, simulate
from arboris.massmatrix import transport, cylinder, box
from arboris.homogeneousmatrix import transl
from arboris.joints import *
from numpy import arange

def add_robot(w, free_floating=False):
    assert isinstance(w, World)
    
    lengths   = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
    masses   = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]

    if free_floating:
        L = lengths[0]/2
        body = Body(mass=box([L, L, L], masses[0]))
        w.add_link(w.ground, FreeJoint(), body)
        frame = body
    else:
        frame = w.ground
        
    for (length, mass) in zip(lengths, masses):
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
    if free_floating:
        w.getjoints()[1].gpos = [3.14159/4.]
    else:
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


#with_weight = True
with_weight = False
free_floating = True

w = ObservableWorld()       
add_robot(w, free_floating)

if with_weight:
    w.register(arboris.controllers.WeightController(w))
    t_end = 2.08
else:
    if free_floating:
        joints = w.getjoints()[1:]
    else:
        joints = w.getjoints()
    for j in joints:
        j.gvel[:] = 2.
    t_end = 1.430

nrj = EnergyMonitor(w)      
w.observers.append(nrj)
mM = MassMonitor(w)
w.observers.append(mM)
w.observers.append(Drawer(w))
timeline = arange(0, t_end, 0.005)
simulate(w, timeline)

nrj.plot()
mM.plot()
    

