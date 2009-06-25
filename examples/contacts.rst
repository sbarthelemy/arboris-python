
Test contact
============

Non sliding, free
-----------------

>>> from arboris.visu_osg import NodeFactory, DrawableWorld
>>> from arboris.robots.simpleshapes import ball
>>> from arboris.collisions import sphere_sphere_collision
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate
>>> from numpy import arange
>>> w = DrawableWorld(factory=NodeFactory(scale=.1))
>>> foo = ball(world=w, name='ball0')
>>> foo = ball(world=w, name='ball1')
>>> bodies = w.getbodiesdict()
>>> bodies['ball0'].parentjoint.gpos[1,3] = 2.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> collision_solver = sphere_sphere_collision
>>> c = SoftFingerContact(w._shapes, 0.1, collision_solver)
>>> w.register(c)
>>> w.init()
>>> w.update_geometric()
>>> w.init_graphic()
>>> time = arange(0,1,1e-3)
>>> world = w
>>> world.init()
>>> previous_t = time[0]
>>> for t in time[1:]:
...     dt = t - previous_t
...     world.update_dynamic()
...     world.update_controllers(dt, t)
...     world.update_constraints(dt)
...     world.update_graphic()
...     world.integrate(dt)
...     previous_t = t    


Non-sliding, blocked
--------------------

>>> from arboris.visu_osg import NodeFactory, DrawableWorld
>>> from arboris.shapes import Sphere
>>> from arboris.robots.simpleshapes import ball
>>> from arboris.collisions import sphere_sphere_collision
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate
>>> from numpy import arange
>>> w = DrawableWorld(factory=NodeFactory(scale=.1))
>>> foo = ball(world=w, name='ball0')
>>> w.register(Sphere(w.ground))
>>> bodies = w.getbodiesdict()
>>> bodies['ball0'].parentjoint.gpos[1,3] = 2.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> collision_solver = sphere_sphere_collision
>>> c = SoftFingerContact(w._shapes, 0.1, collision_solver)
>>> w.register(c)
>>> w.init()
>>> w.update_geometric()
>>> w.init_graphic()
>>> time = arange(0,1,1e-3)
>>> world = w
>>> world.init()
>>> previous_t = time[0]
>>> for t in time[1:]:
...     dt = t - previous_t
...     world.update_dynamic()
...     world.update_controllers(dt, t)
...     world.update_constraints(dt)
...     world.update_graphic()
...     world.integrate(dt)
...     previous_t = t    


Sliding, blocked
----------------

>>> from arboris.visu_osg import NodeFactory, DrawableWorld
>>> from arboris.shapes import Sphere
>>> from arboris.robots.simpleshapes import ball
>>> from arboris.collisions import sphere_sphere_collision
>>> from arboris.constraints import SoftFingerContact
>>> from arboris.core import simulate
>>> from numpy import arange
>>> w = DrawableWorld(factory=NodeFactory(scale=.1))
>>> foo = ball(world=w, name='ball0')
>>> w.register(Sphere(w.ground))
>>> bodies = w.getbodiesdict()
>>> bodies['ball0'].parentjoint.gpos[2,3] = 1.99
>>> bodies['ball0'].parentjoint.gpos[1,3] = 0.5
>>> bodies['ball0'].parentjoint.gvel[4] = -1.
>>> collision_solver = sphere_sphere_collision
>>> c = SoftFingerContact(w._shapes, 0.1, collision_solver)
>>> w.register(c)
>>> w.init()
>>> w.update_geometric()
>>> w.init_graphic()
>>> time = arange(0,1.5,1e-3)
>>> world = w
>>> world.init()
>>> previous_t = time[0]
>>> for t in time[1:]:
...     dt = t - previous_t
...     world.update_dynamic()
...     world.update_controllers(dt, t)
...     world.update_constraints(dt)
...     world.update_graphic()
...     world.integrate(dt)
...     previous_t = t    

