
Test contact
============

Non sliding, free
-----------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.simpleshapes import add_sphere
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate, World
>>> from numpy import arange
>>> world = World()
>>> add_sphere(world, name='ball0')
>>> add_sphere(world, name='ball1')
>>> bodies = world.getbodies()
>>> bodies['ball0'].parentjoint.gpos[1,3] = 2.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> c = SoftFingerContact(world._shapes, 0.1)
>>> world.register(c)
>>> simulate(world,  arange(0,1,1e-3), [Drawer(world)])


Non-sliding, blocked
--------------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.simpleshapes import add_sphere
>>> from arboris.shapes import Sphere
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate, World
>>> from numpy import arange
>>> world = World()
>>> add_sphere(world, name='ball0')
>>> world.register(Sphere(world.ground))
>>> bodies = world.getbodies()
>>> bodies['ball0'].parentjoint.gpos[1,3] = 2.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> c = SoftFingerContact(world._shapes, 0.1)
>>> world.register(c)
>>> simulate(world, arange(0,1,1e-3), [Drawer(world)])


Sliding, blocked
----------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.simpleshapes import add_sphere
>>> from arboris.shapes import Sphere
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate, World
>>> from numpy import arange
>>> world = World()
>>> add_sphere(world, name='ball0')
>>> world.register(Sphere(world.ground))
>>> bodies = world.getbodies()
>>> bodies['ball0'].parentjoint.gpos[2,3] = 1.99
>>> bodies['ball0'].parentjoint.gpos[1,3] = 0.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> c = SoftFingerContact(world._shapes, 0.1)
>>> world.register(c)
>>> simulate(world, arange(0,1,1e-3), [Drawer(world)])


Sphere-box with gravity
-----------------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.simpleshapes import add_sphere
>>> from arboris.shapes import Box
>>> from arboris.collisions import box_sphere_collision
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate, World
>>> from arboris.controllers import WeightController
>>> from numpy import arange
>>> world = World()
>>> world.register(Box(world.ground))
>>> world.register(WeightController())
>>> add_sphere(world, name='ball0')
>>> bodies = world.getbodies()
>>> bodies['ball0'].parentjoint.gpos[1,3] = 2.05
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> collision_solver = box_sphere_collision
>>> c = SoftFingerContact(world._shapes, 0.1)
>>> #c = SoftFingerContact(world._shapes, 0.1, collision_solver)
>>> world.register(c)
>>> simulate(world, arange(0,0.7,1e-3), [Drawer(world)])
