

Display a human
===============

... with a drawer
-----------------

    >>> from arboris.visu_osg import WorldDrawer, init_viewer
    >>> from arboris.robots.human36 import human36
    >>> w = human36()
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


... with a drawable world
-------------------------

    >>> from arboris.visu_osg import DrawableWorld
    >>> from arboris.robots.human36 import human36
    >>> w = DrawableWorld()
    >>> foo = human36(world=w)
    >>> w.update_geometric()
    >>> w.init_graphic()
    >>> joints = w.getjointslist()
    >>> t = 0.
    >>> while not(w.graphic_is_done()):
    ...     joints[1].gpos=[t,t,t]
    ...     joints[2].gpos=[t]
    ...     joints[3].gpos=[t,t]
    ...     w.update_geometric()
    ...     w.update_graphic()
    ...     t += 1./180


... as a plugin
---------------

>>> from arboris.visu_osg import DrawerPlugin
>>> from arboris.robots.human36 import human36
>>> from arboris.core import simulate
>>> from numpy import arange
>>> plugin = DrawerPlugin()
>>> world = human36()
>>> time = arange(0, 1, 1e-3)
>>> simulate(world, time, (plugin,))

