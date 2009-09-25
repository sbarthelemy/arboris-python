
>>> from arboris.core import World, Body
>>> from arboris.joints import *
>>> from numpy import dot
>>> from numpy.linalg import norm

>>> world = World()
>>> Ba = Body()
>>> Rzyx = RzRyRxJoint()
>>> world.add_link(world.ground, Rzyx, Ba)
>>> Bzy = Body(name='zy')
>>> Byx = Body(name='yx')
>>> Bb = Body()
>>> Rz = RzJoint()
>>> world.add_link(world.ground, Rz, Bzy)
>>> Ry = RyJoint()
>>> world.add_link(Bzy, Ry, Byx)
>>> Rx = RxJoint()
>>> world.add_link(Byx, Rx, Bb)
>>> world.init()
>>> (az, ay, ax) = (3.14/6, 3.14/4, 3.14/3)
>>> Rzyx.gpos[:] = (az, ay, ax)
>>> (Rz.gpos[0], Ry.gpos[0], Rx.gpos[0]) = (az, ay, ax)
>>> world.update_dynamic()
>>> Ja = Ba.jacobian[:,0:3]
>>> Jb = Bb.jacobian[:,3:6] 
>>> norm(Ja-Jb) < 1e-10
True


