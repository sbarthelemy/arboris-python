
test with a free joint
======================

>>> from arboris.core import World, Body
>>> from arboris.joints import FreeJoint
>>> from arboris.visu_osg import WorldDrawer, init_viewer, JointIkHandler
>>> world = World()
>>> body = Body(name='free body')
>>> joint = FreeJoint(frames=(world.ground, body), name='free joint')
>>> joint.gpos[0,3] = 0.1
>>> world.register(joint)
>>> world.update_geometric()
>>> drawer = WorldDrawer(world)
>>> viewer = init_viewer(drawer)
>>> kbh = JointIkHandler(drawer, viewer)
>>> viewer.addEventHandler(kbh.__disown__())
>>> viewer.realize()
>>> while not(viewer.done()):
...     world.update_geometric()
...     drawer.update()
...     viewer.frame()


test with human36
=================

>>> from arboris.visu_osg import WorldDrawer, init_viewer, JointIkHandler
>>> from arboris.robots.human36 import human36
>>> world = human36()
>>> world.update_geometric()
>>> drawer = WorldDrawer(world)
>>> viewer = init_viewer(drawer)
>>> kbh = JointIkHandler(drawer, viewer)
>>> viewer.addEventHandler(kbh.__disown__())
>>> viewer.realize()
>>> joints = world.getjoints()
>>> t = 0.
>>> while not(viewer.done()):
...     joints[1].gpos=[t,t,t]
...     joints[2].gpos=[t]
...     joints[3].gpos=[t,t]
...     world.update_geometric()
...     drawer.update()
...     viewer.frame()
...     t += 1./180

