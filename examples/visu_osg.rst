

Display a human
===============

... as a plugin
---------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.human36 import add_human36
>>> from arboris.core import simulate, ObservableWorld
>>> from numpy import arange
>>> world = ObservableWorld()
>>> world.observers.append(Drawer(world))
>>> add_human36(world)
>>> timeline = arange(0, 0.1, 1e-3)
>>> simulate(world, timeline)


... with a drawable world
-------------------------

>>> from arboris.core import ObservableWorld
>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.human36 import add_human36
>>> world = ObservableWorld()
>>> drawer = Drawer(world)
>>> world.observers.append(drawer)
>>> add_human36(world)
>>> joints = world.getjointslist()
>>> t = 0.
>>> dt = 1./180
>>> while not(drawer.done()):
...     joints[1].gpos=[t,t,t]
...     joints[2].gpos=[t]
...     joints[3].gpos=[t,t]
...     world.update_geometric()
...     world._update_observers(dt)
...     t += dt 

