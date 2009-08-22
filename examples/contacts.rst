
Test contact
============

Non sliding, free
-----------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.simpleshapes import add_sphere
>>> from arboris.collisions import sphere_sphere_collision
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate, ObservableWorld
>>> from numpy import arange
>>> world = ObservableWorld()
>>> world.observers.append(Drawer(world, scale=.1))
>>> add_sphere(world, name='ball0')
>>> add_sphere(world, name='ball1')
>>> bodies = world.getbodiesdict()
>>> bodies['ball0'].parentjoint.gpos[1,3] = 2.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> collision_solver = sphere_sphere_collision
>>> c = SoftFingerContact(world._shapes, 0.1, collision_solver)
>>> world.register(c)
>>> simulate(world,  arange(0,1,1e-3))


Non-sliding, blocked
--------------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.simpleshapes import add_sphere
>>> from arboris.shapes import Sphere
>>> from arboris.collisions import sphere_sphere_collision
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate, ObservableWorld
>>> from numpy import arange
>>> world = ObservableWorld()
>>> world.observers.append(Drawer(world, scale=.1))
>>> add_sphere(world, name='ball0')
>>> world.register(Sphere(world.ground))
>>> bodies = world.getbodiesdict()
>>> bodies['ball0'].parentjoint.gpos[1,3] = 2.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> collision_solver = sphere_sphere_collision
>>> c = SoftFingerContact(world._shapes, 0.1, collision_solver)
>>> world.register(c)
>>> simulate(world,  arange(0,1,1e-3))


Sliding, blocked
----------------

>>> from arboris.visu_osg import Drawer
>>> from arboris.robots.simpleshapes import add_sphere
>>> from arboris.shapes import Sphere
>>> from arboris.collisions import sphere_sphere_collision
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate, ObservableWorld
>>> from numpy import arange
>>> world = ObservableWorld()
>>> world.observers.append(Drawer(world, scale=.1))
>>> add_sphere(world, name='ball0')
>>> world.register(Sphere(world.ground))
>>> bodies = world.getbodiesdict()
>>> bodies['ball0'].parentjoint.gpos[2,3] = 1.99
>>> bodies['ball0'].parentjoint.gpos[1,3] = 0.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> collision_solver = sphere_sphere_collision
>>> c = SoftFingerContact(world._shapes, 0.1, collision_solver)
>>> world.register(c)
>>> simulate(world,  arange(0,1,1e-3))
