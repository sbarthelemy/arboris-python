from arboris.all import *
from numpy import arange, dot, eye, sqrt
from itertools import ifilter

batch = True
t_end = 1e-2
w = World()
if batch:
    observers = []
else:
    from arboris.visu_osg import Drawer
    observers = [Drawer(w)]

add_groundplane(w, half_extents=(3., 0.01, 2.) )
add_human36(w)

# set initial position
w.ground.childrenjoints[0].gpos = dot(
        homogeneousmatrix.transl(0,0.03,0), w.ground.childrenjoints[0].gpos)
# add weight
w.register(WeightController())

# add a (single) PD Controller to all the simple joints
ljoints = filter(
        lambda x: isinstance(x, core.LinearConfigurationSpaceJoint),
        w.iterjoints());
n = len(JointsList(ljoints).dof)
kp = 0.1*eye(n)
c = ProportionalDerivativeController(ljoints, kp=kp, kd=2.*sqrt(kp))

# add contacts
shapes=w.getshapes()
for c in constraints.get_all_contacts(w, friction_coeff=.6):
    w.register(c)
timeline = arange(0., t_end, 5e-3)
simulate(w, timeline, observers)

