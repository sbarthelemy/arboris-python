

Display a human
===============

... as a plugin
---------------

>>> from arboris.visu_osg import DrawerObserver
>>> from arboris.robots.human36 import add_human36
>>> from arboris.core import simulate, ObservableWorld
>>> from numpy import arange
>>> world = ObservableWorld()
>>> world.observers.append(DrawerObserver(world))
>>> add_human36(world)
>>> timeline = arange(0, 0.1, 1e-3)
>>> simulate(world, timeline)


... with a drawable world
-------------------------

>>> from arboris.core import ObservableWorld
>>> from arboris.visu_osg import DrawerObserver
>>> from arboris.robots.human36 import add_human36
>>> world = ObservableWorld()
>>> drawer = DrawerObserver(world)
>>> world.observers.append(drawer)
>>> add_human36(world)
>>> joints = world.getjointslist()
>>> t = 0.
>>> while not(drawer.done()):
...     joints[1].gpos=[t,t,t]
...     joints[2].gpos=[t]
...     joints[3].gpos=[t,t]
...     world.update_geometric()
...     world._update_observers(0)
...     t += 1./180


... with a drawer
-----------------

>>> from arboris.core import World
>>> from arboris.visu_osg import WorldDrawer, init_viewer
>>> from arboris.robots.human36 import add_human36
>>> w = World()
>>> add_human36(w)
>>> w.update_geometric()
>>> d = WorldDrawer(w)
>>> viewer = init_viewer(d)
>>> viewer.realize()
>>> joints = w.getjointslist()
>>> t = 0.
>>> while not(viewer.done()):
...     joints[1].gpos=[t,t,t]
...     joints[2].gpos=[t]
...     joints[3].gpos=[t,t]
...     w.update_geometric()
...     d.update()
...     viewer.frame()
...     t += 1./180
