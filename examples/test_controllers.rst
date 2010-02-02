
PD Control
==========

>>> from arboris.controllers import ProportionalDerivativeController
>>> from arboris.robots.simplearm import add_simplearm
>>> from arboris.visu_osg import Drawer
>>> from arboris.core import simulate, World
>>> from numpy import arange, diag, sqrt
>>> world = World()
>>> add_simplearm(world)
>>> joints = world.getjoints()
>>> c = ProportionalDerivativeController(joints, 
...     gpos_des=(3.14/4,3.14/4,3.14/4),
...     kp=diag((1.,1.,1.)),
...     kd=diag((1.,1.,1.))/sqrt(2))
>>> world.register(c)
>>> time = arange(0, 3, 1e-3)
>>> simulate(world, time, [Drawer(world)])

