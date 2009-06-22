
PD Control
==========

>>> from arboris.controllers import ProportionalDerivativeController
>>> from arboris.robots.simplearm import simplearm
>>> from arboris.visu_osg import NodeFactory, DrawerPlugin
>>> from arboris.core import simulate
>>> from numpy import arange, diag, sqrt
>>> world = simplearm()
>>> joints = world.getjointslist()
>>> kp = diag((1.,1.,1.))
>>> kd = diag((1.,1.,1.))/sqrt(2)
>>> c = ProportionalDerivativeController(joints,gpos_des=(3.14/4,3.14/4,3.14/4),kp=kp, kd=kd)
>>> world.register(c)
>>> time = arange(0, 3, 1e-3)
>>> plugin = DrawerPlugin(factory=NodeFactory(scale=.1))
>>> simulate(world, time, (plugin,))

