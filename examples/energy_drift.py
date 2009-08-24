
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
    
    l   = [1., .9, .8, .7, .6, .5, .4 , .3, .2]
    r   = [.1, .09, .08, .07, .06, .05, .04, .03, .02] 
    m   = [1., .9, .8, .7, .6, .5, .4 , .3, .2]
    
    nb_body = 7
    
    ##### build bodies
    bd  = []
    bdframe = []
    for i in range(nb_body):
        Mbi = transport( cylinder(l[i], r[i], m[i]), transl(0., -l[i]/2., 0.) )
        bodyi = Body( name='bd'+str(i), mass= Mbi )
        bd.append( bodyi )
        bdframe.append( SubFrame(bd[-1], transl(0., l[i], 0.)) )
    w.register( bdframe[-1] )
    
    
    ##### build joints
    jt = []
    jt.append( RzJoint(name='first', frames=(w.ground, bd[0] )) ) # link ground and body 0
    for i in range(nb_body-1):
        jointi = RzJoint( name='jt'+str(i), frames=(bdframe[i], bd[i+1]) )
        jt.append( jointi ) 
        w.register(jt[-1])
    
    ##### init config
    jt[0].gpos = [3.14159/4.]
    
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
    

