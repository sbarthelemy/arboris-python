

Display a human
===============

... as a plugin
---------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.human36 import add_human36
>>> from arboris.core import simulate, World
>>> from numpy import arange
>>> world = World()
>>> add_human36(world)
>>> timeline = arange(0, 0.1, 1e-3)
>>> simulate(world, timeline, [Drawer(world)])
