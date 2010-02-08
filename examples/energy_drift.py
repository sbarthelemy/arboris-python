# coding=utf-8
"""
This example shows that arboris creates energy.

We can see it many simulations, by toggling:

- initial velocity,
- gravity,
- the free-floating or fixed base.

"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

import arboris.controllers
from arboris.observers import EnergyMonitor, PerfMonitor
from arboris.visu_osg import Drawer
from arboris.core import World, Body, SubFrame, simulate
from arboris.massmatrix import transport, cylinder, box
from arboris.homogeneousmatrix import transl
from arboris.joints import *
from numpy import arange
from arboris.robots.snake import add_snake

def add_robot(w, gpos, gvel, is_fixed=True):
    assert isinstance(w, World)
    
    
    if is_fixed:
        frame = w.ground
    else:
        L = lengths[0]/2.
        body = Body(mass=box([L, L, L], masses[0]))
        w.add_link(w.ground, FreeJoint(), body)
        frame = body
        
        
    for (length, mass, q, dq) in zip(lengths, masses, gpos, gvel):
        radius = length/10.
        M = transport(cylinder(length, radius, mass), 
                      transl(0., -length/2., 0.))
        body = Body(mass=M)
        joint = RzJoint(gpos=q, gvel=dq)
        w.add_link(frame, joint, body)
        frame = SubFrame(body, transl(0., length, 0.))
    w.register(frame)
    w.init()

    
from arboris.core import Observer
from numpy import linalg, where

class MassMonitor(Observer):
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
is_fixed = False
use_snake = True
w = World()
njoints = 9
lengths   = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
masses   = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
gvel = gvel=[2.]*njoints
gpos = [0, 3.14159/4., 0, 0, 0, 0, 0, 0, 0]
#gpos = [0.]*njoints
if use_snake:
    add_snake(w, njoints, lengths=lengths, masses=masses, gpos=gpos, gvel=gvel, is_fixed=False)
else:
    assert njoints == 9
    add_robot(w, gpos=gpos, gvel=gvel, is_fixed=is_fixed)


if with_weight:
    w.register(arboris.controllers.WeightController())
    t_end = 2.08
else:
    t_end = 1.430

nrj = EnergyMonitor(w)      
mM = MassMonitor(w)
w.observers.append(mM)
#w.observers.append(Drawer(w))
timeline = arange(0, t_end, 0.005)
simulate(w, timeline, (nrj, mM))

nrj.plot()
#mM.plot()
    

